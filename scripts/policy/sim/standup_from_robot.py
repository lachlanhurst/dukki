#!/usr/bin/env python3
"""Start the simulated robot in a pose recorded on the real one and see if the stand-up policy gets out.

The pose file (JSON) holds the robot's trunk gravity (trunk frame), its 14 servo joint angles minus
the default pose (robotd/policy order), and the action robotd was sending there:

    {"gravity": [...3], "q_minus_default": [...14], "action": [...14]}

Every robot is placed with that trunk tilt (random yaw), those joint angles, at rest, a little above the
ground; for --hold seconds the recorded action is sent unchanged (the robot settles into the pose under the
same targets, and the policy's last-action input becomes the robot's); then the policy runs for --run seconds.

Also, with --failures, the plain face-up start is run and the robots that did not get up are described
(trunk gravity, joint angles) next to the recorded pose, to see if the robot's stuck pose is one the
simulated failures end in too.

    DUKKI_KP=1 DUKKI_KD=0.05 uv run python <dukki>/scripts/policy/sim/standup_from_robot.py \\
        --task Mjlab-StandUp-Flat-Backlash-Dukki-Distill --checkpoint .../model_7499.pt --pose stuck.json
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict

import torch

import mjlab_microduck.tasks  # noqa: F401
from mjlab_microduck.tasks.mdp import _env_origin_z, _servo_joint_ids
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from rsl_rl.runners import OnPolicyRunner

UPRIGHT_MM = 105.0
UPRIGHT_DEG = 20.0
NAMES = ["l_hyaw", "l_hroll", "l_hpitch", "l_knee", "l_ankle", "neck", "hpitch", "hyaw", "hroll",
         "r_hyaw", "r_hroll", "r_hpitch", "r_knee", "r_ankle"]


def build(task, envs, seconds, device):
    cfg = load_env_cfg(task)
    cfg.scene.num_envs = envs
    cfg.episode_length_s = seconds + 10.0
    removed = [k for k in cfg.events if "push" in k or "topple" in k]
    for name in removed:
        del cfg.events[name]
    ground = cfg.events["set_ground_state"]
    for key in ("face_down", "face_up", "sitting", "standing"):
        ground.params[f"{key}_prob"] = 1.0 if key == "face_up" else 0.0
    for name in [k for k, t in cfg.curriculum.items()
                 if (t.params or {}).get("event_name") in [*removed, "set_ground_state"]]:
        del cfg.curriculum[name]
    return ManagerBasedRlEnv(cfg=cfg, device=device)


def quat_from_gravity(g, yaw):
    """wxyz quaternions (N,4) of a trunk whose frame sees gravity direction g (3,), turned by yaw (N,)."""
    g = g / g.norm()
    down = torch.tensor([0.0, 0.0, -1.0], device=g.device)
    axis = torch.linalg.cross(g, down)
    s = axis.norm()
    angle = torch.atan2(s, torch.dot(g, down))
    axis = axis / s if s > 1e-9 else torch.tensor([1.0, 0.0, 0.0], device=g.device)
    tilt = torch.cat([torch.cos(angle / 2).view(1), axis * torch.sin(angle / 2)])  # body -> world, g -> down
    yaw_q = torch.stack([torch.cos(yaw / 2), torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(yaw / 2)], 1)
    w1, x1, y1, z1 = yaw_q.unbind(1)
    w2, x2, y2, z2 = tilt
    return torch.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], 1)


def render(env, prefix, count=3):
    """PNGs of the first robots, from four sides, at a fixed distance around the trunk."""
    import mujoco
    from PIL import Image
    m = env.sim.mj_model
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 360, 480)
    for i in range(count):
        d.qpos[:] = env.sim.data.qpos[i].cpu().numpy()
        mujoco.mj_forward(m, d)
        tiles = []
        for az in (0, 90, 180, 270):
            cam = mujoco.MjvCamera()
            cam.lookat[:] = d.qpos[:3]
            cam.distance, cam.azimuth, cam.elevation = 0.45, az, -15
            r.update_scene(d, cam)
            tiles.append(r.render())
        import numpy as np
        Image.fromarray(np.concatenate([np.concatenate(tiles[:2], 1), np.concatenate(tiles[2:], 1)], 0)).save(f"{prefix}_{i}.png")
    print(f"   rendered {prefix}_0..{count - 1}.png")


def judge(robot, g, z, up_at_end, first_up, k, dt, in_last_second):
    tilt = torch.rad2deg(torch.arccos((-g[:, 2] / g.norm(dim=1)).clamp(-1, 1)))
    up = (z > UPRIGHT_MM) & (tilt < UPRIGHT_DEG)
    first_up = torch.where(up & first_up.isnan(), torch.full_like(first_up, (k + 1) * dt), first_up)
    if in_last_second:
        up_at_end &= up
    return first_up, up_at_end


def run(args, pose):
    env = build(args.task, args.envs, args.hold + args.run, args.device)
    rl_cfg = load_rl_cfg(args.task)
    wrapped = RslRlVecEnvWrapper(env, clip_actions=rl_cfg.clip_actions)
    runner = (load_runner_cls(args.task) or OnPolicyRunner)(wrapped, asdict(rl_cfg), device=args.device)
    runner.load(args.checkpoint, map_location=args.device)
    policy = runner.get_inference_policy(device=args.device)
    torch.manual_seed(args.seed)
    wrapped.reset()
    env.reset(seed=args.seed)
    robot = env.scene["robot"]
    servo = _servo_joint_ids(env, robot)
    n, dt, dev = args.envs, env.step_dt, env.device
    jn = [robot.joint_names[j] for j in servo]
    ids = torch.arange(n, device=dev)
    default = robot.data.default_joint_pos[0, servo]

    if pose is not None:
        g = torch.tensor(pose["gravity"], device=dev, dtype=torch.float32)
        q = default + torch.tensor(pose["q_minus_default"], device=dev, dtype=torch.float32)
        act = torch.tensor(pose["action"], device=dev, dtype=torch.float32).expand(n, -1).contiguous()
        yaw = torch.rand(n, device=dev) * 2 * math.pi
        d = env.sim.data
        d.qpos[ids, 2] = args.z + _env_origin_z(env, ids)
        d.qpos[ids, 3:7] = quat_from_gravity(g, yaw)
        d.qvel[ids] = 0.0
        hinge = 7 + torch.arange(d.qpos.shape[1] - 7, device=dev)
        d.qpos[ids.unsqueeze(1), hinge.unsqueeze(0)] = 0.0  # backlash hinges at their middle
        for j, s in enumerate(servo):
            d.qpos[ids, 7 + s] = q[j]
        quat = d.qpos[ids, 3:7].clone()
        with torch.no_grad():
            for _ in range(int(args.hold / dt)):
                obs, *_ = wrapped.step(act)
                if args.pin:  # hold the trunk at the robot's tilt; height and position free
                    d.qpos[ids, 3:7] = quat
                    d.qvel[ids, 3:6] = 0.0
        if args.render:
            render(env, args.render)
        gs = robot.data.projected_gravity_b
        qs = robot.data.joint_pos[:, servo] - default
        print(f"   after the {args.hold:.1f} s hold: gravity median {[round(float(v), 2) for v in gs.median(0).values]}"
              f" (robot {[round(v, 2) for v in pose['gravity']]}); joints off the robot's by median"
              f" {float((qs - (q - default)).abs().median()):.3f} rad, worst joint "
              f"{NAMES[int((qs - (q - default)).abs().median(0).values.argmax())]}")
    else:
        obs = wrapped.get_observations()
        if args.render:
            for _ in range(int(args.hold / dt)):
                with torch.no_grad():
                    obs, *_ = wrapped.step(torch.zeros(n, len(servo), device=dev))
            render(env, args.render)

    first_up = torch.full((n,), float("nan"), device=dev)
    up_at_end = torch.ones(n, dtype=torch.bool, device=dev)
    moved = torch.zeros(n, device=dev)
    steps, last = int(args.run / dt), int(1.0 / dt)
    prev = robot.data.joint_pos[:, servo].clone()
    with torch.no_grad():
        for k in range(steps):
            obs, *_ = wrapped.step(policy(obs))
            z = robot.data.root_link_pos_w[:, 2] * 1000 - _env_origin_z(env, ids) * 1000
            first_up, up_at_end = judge(robot, robot.data.projected_gravity_b, z, up_at_end, first_up, k, dt,
                                        k >= steps - last)
            cur = robot.data.joint_pos[:, servo]
            if k >= steps - last:
                moved += (cur - prev).abs().sum(1)
            prev = cur.clone()
    got = ~first_up.isnan()
    frozen = (~got) & (moved < 0.05)
    print(f"   got up {100 * got.float().mean():5.1f}%  stays up {100 * up_at_end.float().mean():5.1f}%"
          f"  time to upright median {float(first_up[got].median()) if got.any() else float('nan'):.2f} s"
          f"  never up and still in the last second {100 * frozen.float().mean():5.1f}%")
    if args.failures and pose is None:
        fail = ~got
        if fail.any():
            gf = robot.data.projected_gravity_b[fail]
            qf = robot.data.joint_pos[fail][:, servo] - default
            print(f"   {int(fail.sum())} never got up; their end state, median [10%, 90%]:")
            print("     gravity " + "  ".join(f"{float(gf[:, i].median()):+.2f} [{float(gf[:, i].quantile(.1)):+.2f},"
                                             f" {float(gf[:, i].quantile(.9)):+.2f}]" for i in range(3)))
            for i, nm in enumerate(NAMES):
                print(f"     {nm:9s} {float(qf[:, i].median()):+.2f} [{float(qf[:, i].quantile(.1)):+.2f},"
                      f" {float(qf[:, i].quantile(.9)):+.2f}]", end="\n" if i % 2 else "")
            print()
            rp = args.compare
            if rp is not None:
                close = ((gf - torch.tensor(rp["gravity"], device=dev)).norm(dim=1) < 0.3) & (
                    (qf - torch.tensor(rp["q_minus_default"], device=dev)).abs().amax(1) < 0.35)
                print(f"     within 0.3 of the robot's gravity and 0.35 rad of every robot joint: {int(close.sum())} of {int(fail.sum())}")
    env.close()
    return jn


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="Mjlab-StandUp-Flat-Backlash-Dukki-Distill")
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--pose", help="JSON from a robot recording; without it, the plain face-up start")
    ap.add_argument("--failures", action="store_true", help="plain face-up: describe the robots that never got up")
    ap.add_argument("--envs", type=int, default=256)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--run", type=float, default=6.0)
    ap.add_argument("--z", type=float, default=0.08, help="trunk height to place it at [m]")
    ap.add_argument("--pin", action="store_true", help="hold the trunk at the recorded tilt during --hold")
    ap.add_argument("--render", help="PNG prefix: render robots after the hold (or after the plain start settles)")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    pose = json.load(open(args.pose)) if args.pose else None
    args.compare = pose
    jn = run(args, None if args.failures else pose)
    print("   servo order:", " ".join(jn))


if __name__ == "__main__":
    main()
