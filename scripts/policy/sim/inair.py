"""The walk policy at zero command with the trunk held still in the air, feet hanging (22/11/2026),
to compare with the robot held by hand. cd microduck_rl && uv run python ../dukki/scripts/policy/sim/inair.py POLICY.onnx OUT.npz [--clean]"""
import sys
sys.path.insert(0, "scripts")
import numpy as np
import torch
import dukki_eval as de

de.SCENARIOS = {"stand": (0.0, 0.0, 0.0)}
clean = "--clean" in sys.argv
loader = de.load_env_cfg


def load_env_cfg(t):
    cfg = loader(t)
    for k in [k for k in cfg.terminations if k != "time_out"]:
        del cfg.terminations[k]
    if clean:
        cfg.observations["actor"].enable_corruption = False
    return cfg


de.load_env_cfg = load_env_cfg
env, wrapped, runner, cmd = de.build("Mjlab-Velocity-Flat-Dukki", 8, 12.0)
policy = de.OnnxPolicy(sys.argv[1])
wrapped.reset(); env.reset(seed=0); de.pin_commands(env, cmd)
robot = env.scene["robot"]
hold = robot.data.root_link_pose_w.clone()
hold[:, 2] += 0.08
zero = torch.zeros(env.num_envs, 6, device=env.device)
robot.write_root_link_pose_to_sim(hold); robot.write_root_link_velocity_to_sim(zero)
obs = wrapped.get_observations()
O, A = [], []
with torch.no_grad():
    for k in range(600):
        de.pin_commands(env, cmd)
        robot.write_root_link_pose_to_sim(hold); robot.write_root_link_velocity_to_sim(zero)
        a = policy(obs)
        O.append(obs["actor"].cpu().numpy().copy()); A.append(a.cpu().numpy().copy())
        obs, _, _, _ = wrapped.step(a)
O, A = np.stack(O), np.stack(A)
np.savez(sys.argv[2], obs=O, act=A)
o, a = O[100:], A[100:]
print(f"RESULT inair{' clean' if clean else ''}: d_action {np.abs(np.diff(a, axis=0)).mean():.3f} q_std {o[:, :, 6:20].std(0).mean():.3f} "
      f"qd_std {o[:, :, 20:34].std(0).mean():.2f} gyro_std {o[:, :, 0:3].std(0).mean():.3f} height {robot.data.root_link_pos_w[:, 2].mean():.3f}")
