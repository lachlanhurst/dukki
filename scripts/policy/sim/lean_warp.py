#!/usr/bin/env python3
"""The ankle-lean test in training's own simulator (mjlab on MuJoCo Warp, the training task's model,
actuator and contacts), to see whether training's robot stands on the sole meshes' faces or rocks on
them as desktop MuJoCo does (`lean_sim.py`, docs/policy-testing.md section 12).

Half the robots lean backwards, half forwards, at --rate deg/s on both ankles after --settle s
standing at home with every target at home, no policy. Randomisation and pushes are taken out, so
every robot is the nominal one. Prints the trunk's lean after settling and where each robot started to tip
(found as lean_sim and the robot analysis find it).

Results 23/11/2026, kp 4, against the robot (lean_test) and desktop MuJoCo on flat boxes (lean_sim):

    model                             settled lean   tips back at   tips forwards at  (ankle reads)
    robot                             -1.2 to +0.3   -0.5 to -0.9   about +9
    Velocity-Flat-Dukki (meshes)      -0.04          -2.1           +12.2
    Velocity-Flat-Backlash-Dukki      -2.4           -1.1           +8.4 (most +9 to +9.7)
    lean_sim, flat boxes              +0.3           -4.9           +13.2

Training's simulator on the sole meshes already stands the robot as if the heel were about 5 mm
shorter than the flat boxes, and with the gear play it matches the robot.

    DUKKI_KP=4 DUKKI_KD=0.05 uv run python ~/not_work/dukki/runs/lean_warp.py [--task Mjlab-Velocity-Flat-Dukki]
"""
from __future__ import annotations

import argparse
import math

import torch

import mjlab_microduck.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

TIP_RATE = 1.0
TIP_TICKS = 3
# Positive lean is forwards: left_ankle down by it, right_ankle up (lean_test, lean_sim).
ANKLE_SIGN = {"left_ankle": -1.0, "right_ankle": 1.0}


def build(task: str, envs: int, seconds: float, device: str) -> ManagerBasedRlEnv:
    cfg = load_env_cfg(task)
    cfg.scene.num_envs = envs
    cfg.episode_length_s = seconds + 5.0
    # Keep the resets and the actuator's own setup (bam_init); drop every randomisation and push.
    removed = [k for k in cfg.events if not (k.startswith("reset") or k in ("bam_init", "expand_bam_friction_fields"))]
    for name in removed:
        del cfg.events[name]
    for name in [k for k, t in cfg.curriculum.items() if (t.params or {}).get("event_name") in removed]:
        del cfg.curriculum[name]
    print("events kept:", sorted(cfg.events), "\nevents removed:", sorted(removed))
    return ManagerBasedRlEnv(cfg=cfg, device=device)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="Mjlab-Velocity-Flat-Dukki")
    ap.add_argument("--envs", type=int, default=16)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--max-deg", type=float, default=20.0)
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()

    seconds = args.settle + args.max_deg / args.rate
    env = build(args.task, args.envs, seconds, args.device)
    env.reset(seed=0)
    robot = env.scene["robot"]
    term = env.action_manager.get_term("joint_pos")
    names = list(term.target_names)
    scale = term.scale if torch.is_tensor(term.scale) else torch.full((args.envs, len(names)), float(term.scale), device=env.device)
    offset = term.offset if torch.is_tensor(term.offset) else torch.full((args.envs, len(names)), float(term.offset), device=env.device)
    home = robot.data.default_joint_pos[:, term.target_ids].clone()
    ankle_cols = {n: names.index(n) for n in ANKLE_SIGN}
    ankle_jids = {n: robot.joint_names.index(n) for n in ANKLE_SIGN}
    n, dt = args.envs, env.step_dt
    direction = torch.where(torch.arange(n, device=env.device) < n // 2, -1.0, 1.0)
    print(f"{args.task}: {n} robots, step {dt * 1000:.0f} ms, first half lean backwards, second half forwards")

    def act(delta_deg: torch.Tensor) -> torch.Tensor:
        target = home.clone()
        for nm, col in ankle_cols.items():
            target[:, col] += torch.deg2rad(ANKLE_SIGN[nm] * delta_deg)
        return (target - offset) / scale

    def lean() -> torch.Tensor:
        g = robot.data.projected_gravity_b
        return torch.rad2deg(torch.asin((g[:, 0] / g.norm(dim=1)).clamp(-1, 1)))

    settle_steps = round(args.settle / dt)
    with torch.no_grad():
        for _ in range(settle_steps):
            env.step(act(torch.zeros(n, device=env.device)))
    settled = lean()
    print(f"after {args.settle} s at home: trunk lean {float(settled.mean()):+.2f} deg "
          f"(min {float(settled.min()):+.2f}, max {float(settled.max()):+.2f}); "
          f"trunk height {float(robot.data.root_link_pos_w[:, 2].mean()) * 1000:.0f} mm")

    hist_rate, hist_reads, hist_lean = [], [], []
    tipped = torch.zeros(n, dtype=torch.bool, device=env.device)
    fast = torch.zeros(n, device=env.device)
    tip_ankle = torch.full((n,), float("nan"), device=env.device)
    tip_lean = torch.full((n,), float("nan"), device=env.device)
    delta = torch.zeros(n, device=env.device)
    with torch.no_grad():
        for k in range(round(args.max_deg / args.rate / dt)):
            delta = torch.where(tipped, delta, direction * (k + 1) * dt * args.rate)
            env.step(act(delta))
            rate = robot.data.root_link_ang_vel_b[:, 1].abs()
            q = robot.data.joint_pos
            hist_rate.append(rate.clone())
            hist_reads.append(sum(ANKLE_SIGN[nm] * torch.rad2deg(q[:, j] - robot.data.default_joint_pos[:, j])
                                  for nm, j in ankle_jids.items()) / 2)
            hist_lean.append(lean())
            fast = torch.where(rate > TIP_RATE, fast + 1, torch.zeros_like(fast))
            now = (fast >= TIP_TICKS) & ~tipped
            if now.any():
                q = robot.data.joint_pos
                reads = sum(ANKLE_SIGN[nm] * torch.rad2deg(q[:, j] - robot.data.default_joint_pos[:, j])
                            for nm, j in ankle_jids.items()) / 2
                tip_ankle = torch.where(now, reads, tip_ankle)
                tip_lean = torch.where(now, lean(), tip_lean)
                tipped |= now
            if tipped.all():
                break
    # The onset, as lean_sim and the robot analysis take it: back from the detection to the last
    # tick under 0.05 rad/s.
    R, A, L = (torch.stack(h).cpu() for h in (hist_rate, hist_reads, hist_lean))
    for e in range(n):
        if not bool(tipped[e]):
            continue
        k = int(((R[:, e] > TIP_RATE).float().cumsum(0) >= 1).nonzero()[0])
        while k > 0 and R[k, e] >= 0.05:
            k -= 1
        tip_ankle[e], tip_lean[e] = float(A[k, e]), float(L[k, e])
    for word, sel in (("backwards", direction < 0), ("forwards", direction > 0)):
        got = sel & tipped
        if got.any():
            print(f"{word}: {int(got.sum())} of {int(sel.sum())} tipped, ankles read {float(tip_ankle[got].mean()):+.2f} deg "
                  f"(spread {float(tip_ankle[got].min()):+.2f} to {float(tip_ankle[got].max()):+.2f}), "
                  f"trunk lean {float(tip_lean[got].mean()):+.2f} deg, at the onset")
        else:
            print(f"{word}: none tipped by {args.max_deg} deg")
    env.close()


if __name__ == "__main__":
    main()
