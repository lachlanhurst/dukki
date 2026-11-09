"""Render a MuJoCo video of the J288 BAM model replaying a recorded pendulum log.

The simulation is driven by the exact commands the real servo was sent in the
log (target, speed and torque commands, gains, torque on/off), so the video can
be laid side by side with footage of the real arm filmed during that recording.
Record with ``python -m bam.unitree.record ... --sync`` to start the log with a
sharp flick: it happens at 0.5 s in both videos, which gives a frame to line
them up on.

    uv run --with-editable ../bam --with mujoco --with imageio --with imageio-ffmpeg --with pillow \\
        scripts/j288/render_video.py ../bam/data_raw/video/<log>.json --out arm_sim.mp4 --fps 60

The camera looks along the servo shaft. If the simulated arm swings the opposite
way to the footage, the phone filmed it from the other side: add ``--mirror``.
``--ghost`` overlays the real arm's logged angle as a translucent arm.

``--profile`` renders a profile before it is recorded, from the commands
``bam.unitree.record`` would send, so it lines up with a later recording made
with the same options (it starts at the profile's first target, at rest)::

    uv run ... scripts/j288/render_video.py --profile sin_sin --kp 2.01 --kd 0.05 --sync \\
        --duration 12 --length 0.15 --mass 0.316 --arm-mass 0.017 --mass-inertia 0.000119 \\
        --out arm_sim.mp4
"""

import argparse
import json

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from bam.model import load_model
from bam.mujoco import MujocoController
from bam.testbench_mujoco import Pendulum
from bam.unitree.digital_servo import sent_gains
from bam.unitree.record import SyncPrefix, make_profile

# The end mass is drawn as a 50 mm square turned 45 degrees. Drawing only:
# mass, inertia and gravity come from the log (the real discs).
HALF_SIDE = 0.025  # half the square's side
DIAMOND = 2 * HALF_SIDE * np.sqrt(2)  # corner to corner
STEEL = 7850.0  # kg/m^3, for the drawn block's thickness
TURN_45 = [np.cos(np.pi / 8), np.sin(np.pi / 8), 0.0, 0.0]  # 45 degrees about the shaft


def build_scene(log: dict, q_offset: float, ghost: bool):
    length, mass = log["length"], log["mass"]
    spec = Pendulum({"mass": mass, "arm_mass": log.get("arm_mass", 0.0), "length": length}).build_spec(
        "pendulum", q_offset=q_offset
    )
    body = spec.body("pendulum")
    body.inertia = np.asarray(body.inertia) + log.get("mass_inertia", 0.0)

    # Visuals for this rig: a flat arm, the end mass (drawn as a diamond) swinging
    # flat, and the servo and bracket behind the arm.
    thickness = max(0.004, mass / (STEEL * (2 * HALF_SIDE) ** 2))
    for g in spec.geoms:
        if g.name == "arm":
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = [0.003, 0.009, length / 2 + 0.008]
            g.pos = [0.0, 0.0, -length / 2]
            g.fromto = [np.nan] * 6
            g.rgba = [0.12, 0.12, 0.13, 1.0]
        elif g.name == "floor":
            # Just below the disc's lowest reach (BAM's scene leaves 120 mm).
            g.pos = [0.0, 0.0, -(length + DIAMOND / 2 + 0.015)]
        elif g.name == "bob":
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.fromto = [np.nan] * 6
            g.size = [thickness / 2, HALF_SIDE, HALF_SIDE]
            g.pos = [-0.003 - thickness / 2, 0.0, -length]
            g.quat = TURN_45
            g.rgba = [0.72, 0.72, 0.70, 1.0]
    spec.worldbody.add_geom(
        name="servo", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.013, 0.010, 0.017],
        pos=[0.016, 0.0, -0.006], rgba=[0.20, 0.20, 0.22, 1.0], contype=0, conaffinity=0,
    )
    spec.worldbody.add_geom(
        name="bracket", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.004, 0.022, 0.030],
        pos=[0.033, 0.0, 0.004], rgba=[0.10, 0.10, 0.11, 1.0], contype=0, conaffinity=0,
    )
    if ghost:
        real = spec.worldbody.add_body(name="real", mocap=True, pos=[0, 0, 0])
        real.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.0031, 0.0091, length / 2 + 0.008], pos=[0, 0, -length / 2],
            rgba=[0.16, 0.47, 0.84, 0.35], contype=0, conaffinity=0,
        )
        real.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX, size=[thickness / 2 + 0.0002, HALF_SIDE + 0.0005, HALF_SIDE + 0.0005],
            pos=[-0.0031 - thickness / 2, 0.0, -length], quat=TURN_45,
            rgba=[0.16, 0.47, 0.84, 0.35], contype=0, conaffinity=0,
        )
    model = spec.compile()
    model.opt.timestep = 0.001
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    return model


def profile_log(args) -> dict:
    """A log of the commands bam.unitree.record would send for a profile, at 1 kHz."""
    profile = make_profile(args.profile, args.kp, args.kd, args.amplitude)
    if args.duration is not None:
        profile.duration = args.duration
    if args.sync:
        profile = SyncPrefix(profile, args.kp, args.kd)
    entries = []
    for t in np.arange(0.0, profile.duration, 0.001):
        cmd = profile(t)
        cmd["kp"], cmd["kd"] = sent_gains(cmd["kp"], cmd["kd"])
        entries.append({**cmd, "timestamp": float(t)})
    entries[0]["position"] = entries[0]["goal_position"] if entries[0]["torque_enable"] else 0.0
    return {
        "profile": args.profile, "length": args.length, "mass": args.mass, "arm_mass": args.arm_mass,
        "mass_inertia": args.mass_inertia, "sync": args.sync, "entries": entries,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="?", help="a raw recording from bam.unitree.record")
    p.add_argument("--out", required=True, help="MP4 to write")
    p.add_argument("--model", default="m6", help="bundled J288 model, m1 to m6")
    p.add_argument("--params", help="a parameter file instead of the bundled model")
    p.add_argument("--fps", type=float, default=60.0, help="match the footage's frame rate")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--mirror", action="store_true", help="view from the other side of the servo")
    p.add_argument("--elevation", type=float, default=0.0, help="camera elevation [deg]")
    p.add_argument("--ghost", action="store_true", help="overlay the real arm's logged angle")
    p.add_argument("--label", default=None, help="caption, default from the log")
    g = p.add_argument_group("instead of a log, a profile as bam.unitree.record would play it")
    g.add_argument("--profile", help="e.g. sin_sin, lift_and_damp, torque_chirp")
    g.add_argument("--kp", type=float, default=0.97)
    g.add_argument("--kd", type=float, default=0.02)
    g.add_argument("--amplitude", type=float)
    g.add_argument("--sync", action="store_true")
    g.add_argument("--duration", type=float)
    g.add_argument("--length", type=float, default=0.15, help="pivot to mass centre [m]")
    g.add_argument("--mass", type=float, default=0.316, help="end mass [kg]")
    g.add_argument("--arm-mass", type=float, default=0.017, help="[kg]")
    g.add_argument("--mass-inertia", type=float, default=0.000119, help="end mass's own inertia [kg.m^2]")
    args = p.parse_args()
    if (args.log is None) == (args.profile is None):
        p.error("give a log or --profile, not both")
    if args.ghost and args.log is None:
        p.error("--ghost needs a recorded log")

    log = json.load(open(args.log)) if args.log else profile_log(args)
    entries = sorted(log["entries"], key=lambda e: e["timestamp"])
    t_log = np.array([e["timestamp"] for e in entries])
    q_real = np.array([e.get("position", np.nan) for e in entries])

    bam_model = load_model(args.params) if args.params else load_model(motor_name="unitree_j288", model=args.model)
    act = bam_model.actuator
    act.vin = log.get("vin", 24.0)
    q_offset = bam_model.q_offset.value

    model = build_scene(log, q_offset, args.ghost)
    data = mujoco.MjData(model)
    ctrl = MujocoController(bam_model, "pendulum", model, data)
    data.qpos[0] = entries[0]["position"]
    data.qvel[0] = entries[0].get("speed", 0.0)
    mujoco.mj_forward(model, data)
    ctrl.reset()

    # The offscreen buffer must be at least the video size.
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    renderer = mujoco.Renderer(model, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, -log["length"] * 0.45]
    # Far enough that the vertical framing holds the swing, and that the arm
    # held horizontal still clears the sides with a margin at any aspect ratio.
    reach = log["length"] + DIAMOND / 2
    half_fov = np.tan(np.radians(model.vis.global_.fovy / 2))
    cam.distance = max(2.6 * log["length"] + 0.18, 1.25 * reach / (half_fov * args.width / args.height))
    cam.azimuth = 180.0 if args.mirror else 0.0
    cam.elevation = args.elevation

    label = args.label or (
        f"Simulation, J288 BAM {bam_model.name.upper()}: {log['length'] * 1000:.0f} mm arm, "
        f"{(log['mass'] + log.get('arm_mass', 0)) * 1000:.0f} g, {log.get('profile', '')}"
    )
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", max(18, args.height // 30))
    except OSError:
        font = ImageFont.load_default()

    writer = imageio.get_writer(args.out, fps=args.fps, codec="libx264", quality=8, pixelformat="yuv420p", macro_block_size=8)
    duration = t_log[-1]
    k = 0  # index of the log entry whose command is in force
    n_frames = int(np.floor(duration * args.fps)) + 1
    for f in range(n_frames):
        t_frame = f / args.fps
        while data.time < t_frame - 1e-9:
            while k + 1 < len(entries) and entries[k + 1]["timestamp"] <= data.time:
                k += 1
            e = entries[k]
            act.kp, act.kd = e["kp"], e["kd"]
            act.tau_ff, act.w_des = e.get("goal_torque", 0.0), e.get("goal_speed", 0.0)
            act.enabled = bool(e["torque_enable"])
            ctrl.q_target[:] = e["goal_position"]
            ctrl.update()
            mujoco.mj_step(model, data)
        if args.ghost:
            angle = np.interp(t_frame, t_log, q_real) + q_offset
            data.mocap_quat[0] = [np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0]
            mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        img = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(img)
        pad = args.height // 36
        draw.text((pad, pad), label, fill=(235, 235, 230), font=font)
        if args.ghost:
            draw.text((pad, pad * 2.6), "solid: simulated    blue: real servo (logged angle)", fill=(150, 190, 240), font=font)
        draw.text((args.width - pad * 9, args.height - pad * 2.2), f"t = {t_frame:5.2f} s", fill=(235, 235, 230), font=font)
        writer.append_data(np.asarray(img))
    writer.close()
    print(f"wrote {args.out}: {n_frames} frames at {args.fps:g} fps, {duration:.2f} s")
    if log.get("sync"):
        print(f"sync flick starts at 0.50 s: frame {round(0.5 * args.fps)}")


if __name__ == "__main__":
    main()
