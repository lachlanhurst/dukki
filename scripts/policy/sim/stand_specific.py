"""Sim stand and walk with the robot's suspected calibration errors set exactly (22/11/2026):
left hip roll reading off by 3.4 deg, IMU pitch off by 5 deg, both, and neither; each with both signs.
cd microduck_rl && uv run python ../dukki/scripts/policy/sim/stand_specific.py POLICY.onnx"""
import sys, math
sys.path.insert(0, "scripts")
import numpy as np
import torch
import dukki_eval as de

de.SCENARIOS = {"stand": (0.0, 0.0, 0.0), "walk_0.15": (0.15, 0.0, 0.0)}
CASES = [("none", 0, 0), ("hroll+3.4", 3.4, 0), ("hroll-3.4", -3.4, 0), ("imu+5", 0, 5), ("imu-5", 0, -5),
         ("both++", 3.4, 5), ("both+-", 3.4, -5), ("both-+", -3.4, 5)]
env, wrapped, runner, cmd = de.build("Mjlab-Velocity-Flat-Dukki", len(CASES), 12.0)
policy = de.OnnxPolicy(sys.argv[1])
wrapped.reset(); env.reset(seed=0); de.pin_commands(env, cmd)
robot = env.scene["robot"]
j = list(robot.joint_names).index("left_hip_roll")
n = env.num_envs
bias = robot.data.encoder_bias
bias[:] = 0.0
q = torch.zeros(n, 4, device=env.device); q[:, 0] = 1.0
for s in range(2):
    for c, (_, hr, ip) in enumerate(CASES):
        e = s * len(CASES) + c
        bias[e, j] = math.radians(hr)
        a = math.radians(ip) / 2
        q[e] = torch.tensor([math.cos(a), 0.0, math.sin(a), 0.0])
env._imu_misalign_quat = q
obs = wrapped.get_observations()
O, A, F = [], [], np.zeros(n)
with torch.no_grad():
    for k in range(600):
        de.pin_commands(env, cmd)
        act = policy(obs)
        O.append(obs["actor"].cpu().numpy().copy()); A.append(act.cpu().numpy().copy())
        obs, _, dones, extras = wrapped.step(act)
        to = extras.get("time_outs", torch.zeros_like(dones))
        if k >= 100:
            F += (dones.bool() & ~to.bool()).cpu().numpy()
O, A = np.stack(O)[100:], np.stack(A)[100:]
for s, sc in enumerate(de.SCENARIOS):
    for c, (name, _, _) in enumerate(CASES):
        e = s * len(CASES) + c
        o, a = O[:, e], A[:, e]
        print(f"RESULT {sc:10} {name:10} falls {int(F[e])} d_action {np.abs(np.diff(a, axis=0)).mean():.3f} "
              f"q_std {o[:, 6:20].std(0).mean():.3f} gyro_std {o[:, 0:3].std(0).mean():.3f}")
