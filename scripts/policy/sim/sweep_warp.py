#!/usr/bin/env python3
"""The joint sweep (microduck `duck-control/examples/joint_sweep`) in training's own simulator, to
read the robot's joint friction as a multiple of the BAM bench fit that training scales.

The trunk is held fixed in the air (its free joint taken out), legs hanging, as the robot is held by
hand. Every robot gets the same sweep: home, then for each frequency the left joint swings ±amp for
`cycles` cycles (faded in and out over a cycle each), then the right one mirrored, 1.5 s of hold
between. Robot k runs with every joint's friction at `--scales[k]` times the bench fit, the rest
of the randomisation taken out. Each robot's record is written in joint_sweep's format, so
`scripts/policy/sweep_summary.py` reads it exactly as it reads the robot's.

    cd microduck_rl && DUKKI_KP=1 DUKKI_KD=0.07 uv run python ../dukki/scripts/policy/sim/sweep_warp.py \
        --joint hip_yaw --scales 0.5,1,2,3,4 --out ../dukki/scratch/sweep_sim [--device cpu]
"""
from __future__ import annotations

import argparse
import json
import math
import os

import mujoco
import torch

import mjlab_microduck.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

# joint_sweep's order (the bridge's): the arrays in the record follow it.
NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch",
         "head_pitch", "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll",
         "right_hip_pitch", "right_knee", "right_ankle"]
HOME_S = 4.5  # joint_sweep's 3 s ramp and 1.5 s hold; the simulation starts at home
HOLD_S = 1.5
LIFT_M = 0.15  # trunk raised this far above its standing height, feet clear of the floor


def fixed_base(spec_fn):
    """spec_fn with the root's free joint removed: mjlab then holds the trunk where the reset puts it."""
    def wrapped() -> mujoco.MjSpec:
        spec = spec_fn()
        for j in list(spec.joints):
            if j.type == mujoco.mjtJoint.mjJNT_FREE:
                spec.delete(j)
        return spec
    return wrapped


def build(task: str, envs: int, seconds: float, device: str) -> ManagerBasedRlEnv:
    cfg = load_env_cfg(task)
    cfg.scene.num_envs = envs
    cfg.episode_length_s = seconds + 5.0
    removed = [k for k in cfg.events if not (k.startswith("reset") or k in ("bam_init", "expand_bam_friction_fields"))]
    for name in removed:
        del cfg.events[name]
    for name in [k for k, t in cfg.curriculum.items() if (t.params or {}).get("event_name") in removed]:
        del cfg.curriculum[name]
    for name in [k for k in cfg.terminations if k != "time_out"]:
        del cfg.terminations[name]
    robot = cfg.scene.entities["robot"]
    robot.spec_fn = fixed_base(robot.spec_fn)
    pos = list(robot.init_state.pos)
    pos[2] += LIFT_M
    robot.init_state.pos = tuple(pos)
    # Every reset puts the trunk at its initial pose, level and unrandomised.
    for name, term in cfg.events.items():
        params = term.params or {}
        for key in ("pose_range", "velocity_range", "position_range"):
            if key in params:
                params[key] = {} if isinstance(params[key], dict) else (0.0, 0.0)
    print("events kept:", sorted(cfg.events))
    return ManagerBasedRlEnv(cfg=cfg, device=device)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="Mjlab-Velocity-Flat-Backlash-Dukki")
    ap.add_argument("--joint", default="hip_yaw")
    ap.add_argument("--amp", type=float, default=15.0)
    ap.add_argument("--freqs", default="0.5,2")
    ap.add_argument("--cycles", type=float, default=4.0)
    ap.add_argument("--scales", default="0.5,1,2,3,4")
    ap.add_argument("--out", default="sweep_sim")
    ap.add_argument("--device", default="cuda:0" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    freqs = [float(f) for f in args.freqs.split(",")]
    scales = [float(s) for s in args.scales.split(",")]
    plan = [("home", HOME_S, None)]
    for f in freqs:
        for side, sign in (("left", 1.0), ("right", -1.0)):
            label = f"{side} {int(f) if f.is_integer() else f} Hz"
            plan += [(label, (args.cycles + 2.0) / f, (f"{side}_{args.joint}", sign, f)), ("hold", HOLD_S, None)]
    seconds = sum(p[1] for p in plan)

    n = len(scales)
    env = build(args.task, n, seconds, args.device)
    env.reset(seed=0)
    robot = env.scene["robot"]
    scale_t = torch.tensor(scales, device=env.device)
    for act in robot.actuators:
        if getattr(act, "friction_scale", None) is not None:
            joints = len(act._dof_ids)
            act.friction_scale = scale_t[:, None].expand(-1, joints).clone()
            print(f"{type(act).__name__}: friction scale per robot {scales} on {joints} joints")
    term = env.action_manager.get_term("joint_pos")
    names = list(term.target_names)
    scale = term.scale if torch.is_tensor(term.scale) else torch.full((n, len(names)), float(term.scale), device=env.device)
    offset = term.offset if torch.is_tensor(term.offset) else torch.full((n, len(names)), float(term.offset), device=env.device)
    home = robot.data.default_joint_pos[:, term.target_ids].clone()
    act_col = {nm: names.index(nm) for nm in names}
    jid = {nm: robot.joint_names.index(nm) for nm in NAMES if nm in robot.joint_names}
    kp = float(os.environ.get("DUKKI_KP", "nan"))
    kd = float(os.environ.get("DUKKI_KD", "nan"))
    dt = env.step_dt
    amp = math.radians(args.amp)
    print(f"{args.task}: {n} robots held in the air, step {dt * 1000:.0f} ms, {seconds:.0f} s of sweep")

    def torque_of(j: int) -> torch.Tensor:
        """The motor torque the servo would report for joint j: what the actuator applies."""
        return robot.data.data.qfrc_actuator[:, robot.data.indexing.joint_v_adr[j]]

    frames = [[] for _ in range(n)]
    t = 0.0
    with torch.no_grad():
        for label, length, swing in plan:
            for k in range(round(length / dt)):
                target = home.clone()
                if swing is not None:
                    name, sign, f = swing
                    c = (k + 1) * dt * f
                    envelope = min(c, 1.0, max(args.cycles + 2.0 - c, 0.0))
                    target[:, act_col[name]] += sign * amp * envelope * math.sin(2 * math.pi * c)
                env.step((target - offset) / scale)
                t += dt
                q = robot.data.joint_pos
                tau = {nm: torque_of(j) for nm, j in jid.items()}
                for e in range(n):
                    tg = [float(target[e, act_col[nm]]) if nm in act_col else 0.0 for nm in NAMES]
                    pos = [float(q[e, jid[nm]]) if nm in jid else 0.0 for nm in NAMES]
                    frames[e].append({
                        "t": round(t, 3), "phase": label, "targets": tg, "positions": pos,
                        "torques": [float(tau[nm][e]) if nm in tau else 0.0 for nm in NAMES],
                        # No output encoder here: the joint angle in counts, so output/rotor reads 1.
                        "encoders": [round(p * 8192 / (2 * math.pi)) for p in pos],
                    })
    os.makedirs(args.out, exist_ok=True)
    for e, s in enumerate(scales):
        path = os.path.join(args.out, f"sweep_{args.joint}_x{s:g}.json")
        with open(path, "w") as fh:
            json.dump({"joint": args.joint, "amp_deg": args.amp, "kp": kp, "kd": kd, "freqs": freqs,
                       "cycles": args.cycles, "friction_scale": s, "task": args.task, "frames": frames[e]}, fh)
        print("wrote", path)
    env.close()


if __name__ == "__main__":
    main()
