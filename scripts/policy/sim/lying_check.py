"""How the simulated robot lies at HOME, face up and face down (no roll noise), against the robot's readings.

Run with DUKKI_BODY_CONTACT=0 and =1. Robot, 23/11/2026, at HOME after robotd's ramp: face up gravity
(-0.97, -0.04, -0.24), face down (+0.99, 0.00, -0.13) (trunk frame).
"""
import math, sys
import torch
import importlib.util
import os
spec = importlib.util.spec_from_file_location("sfr", os.path.join(os.path.dirname(os.path.abspath(__file__)), "standup_from_robot.py"))
sfr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sfr)
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

task = "Mjlab-StandUp-Flat-Backlash-Dukki-Distill"
for start in ("face_up", "face_down"):
    cfg = load_env_cfg(task)
    cfg.scene.num_envs = 64
    cfg.episode_length_s = 20
    removed = [k for k in cfg.events if "push" in k or "topple" in k]
    for k in removed:
        del cfg.events[k]
    g = cfg.events["set_ground_state"]
    for key in ("face_down", "face_up", "sitting", "standing"):
        g.params[f"{key}_prob"] = 1.0 if key == start else 0.0
    g.params["face_up_roll_max"] = 0.0
    for k in [k for k, t in cfg.curriculum.items() if (t.params or {}).get("event_name") in [*removed, "set_ground_state"]]:
        del cfg.curriculum[k]
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    env.reset(seed=0)
    robot = env.scene["robot"]
    peak = torch.zeros(64, device=env.device)
    with torch.no_grad():
        for k in range(100):
            env.step(torch.zeros(64, 14, device=env.device))
            peak = torch.maximum(peak, robot.data.root_link_ang_vel_b.norm(dim=1))
    gv = robot.data.projected_gravity_b
    z = robot.data.root_link_pos_w[:, 2]
    print(f"== {start}: gravity median {[round(float(v), 2) for v in gv.median(0).values]}"
          f" [10% {[round(float(v), 2) for v in gv.quantile(0.1, 0)]}, 90% {[round(float(v), 2) for v in gv.quantile(0.9, 0)]}]"
          f" trunk z median {1000 * float(z.median()):.0f} mm; peak body rate while settling median {float(peak.median()):.1f} rad/s, max {float(peak.max()):.1f}")
    if len(sys.argv) > 1:
        sfr.render(env, f"{sys.argv[1]}_{start}", count=1)
    env.close()
