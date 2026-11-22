"""The robot walk test (scripts/policy/walk_test.py) in simulation: 32 robots stand 4 s, ramp to vx
over 1 s, hold it 6 s, ramp down over 1 s. Prints falls, the worst tilt and the mean action change
per phase. Runs from a microduck_rl checkout (it uses scripts/dukki_eval.py's setup: the training
randomisation kept, pushes off).

    cd microduck_rl
    DUKKI_KP=1 DUKKI_KD=0.05 DUKKI_CALIBRATION_SPREAD=0 DUKKI_FRICTION=3,5 \\
        uv run python /path/to/walk_profile.py POLICY.onnx|model_N.pt [vx]

Switches (environment):
    DUKKI_FRICTION=lo,hi   per-joint friction scale on the bench fit (the task's own switch)
    DUKKI_KD=kd            the servos' kd (the task's own switch)
    ROBOT_FILTERS=1        robotd's target low-pass: legs LEGS_ALPHA (default 0.7), head 0.5
    LEGS_ALPHA=a           with ROBOT_FILTERS, the legs' alpha
    IMU_PITCH_DEG=d        a fixed IMU mounting error about the trunk's y, in place of the random one

Never set the training-only switches (DUKKI_FRICTION_RAMP and the rest) here: a ramp would move
the friction while the test runs.
"""
import math
import os
import sys

sys.path.insert(0, "scripts")
import numpy as np  # noqa: E402
import torch  # noqa: E402
import dukki_eval as de  # noqa: E402

VX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
N = 32
de.SCENARIOS = {"profile": (0.0, 0.0, 0.0)}

if os.environ.get("ROBOT_FILTERS"):
    from dataclasses import fields

    from mjlab_microduck.tasks.lowpass_action import LowPassJointPositionActionCfg

    legs = float(os.environ.get("LEGS_ALPHA", "0.7"))
    _load = de.load_env_cfg

    def _filtered(task):
        cfg = _load(task)
        a = cfg.actions["joint_pos"]
        kept = {f.name: getattr(a, f.name) for f in fields(a) if f.init and f.name != "lowpass"}
        cfg.actions["joint_pos"] = LowPassJointPositionActionCfg(
            **kept, lowpass={r".*(hip|knee|ankle).*": legs, r"(neck|head)_.*": 0.5})
        return cfg

    de.load_env_cfg = _filtered

env, wrapped, runner, cmd = de.build("Mjlab-Velocity-Flat-Dukki", N, 20.0)
if sys.argv[1].endswith(".onnx"):
    policy = de.OnnxPolicy(sys.argv[1])
else:
    runner.load(sys.argv[1], map_location=env.device)
    policy = runner.get_inference_policy(device=env.device)
wrapped.reset()
env.reset(seed=0)
robot = env.scene["robot"]

pitch = float(os.environ.get("IMU_PITCH_DEG", "0"))
if pitch:
    half = math.radians(pitch) / 2
    q = torch.zeros(env.num_envs, 4, device=env.device)
    q[:, 0], q[:, 2] = math.cos(half), math.sin(half)
    env._imu_misalign_quat = q


def vx_at(t):
    t -= 4.0
    if t < 0:
        return 0.0
    if t < 1:
        return VX * t
    if t < 7:
        return VX
    if t < 8:
        return VX * (8 - t)
    return 0.0


obs = wrapped.get_observations()
fell = np.zeros(N, bool)
fell_at = np.full(N, np.nan)
tilt_max = {"stand": 0.0, "walk": 0.0, "down": 0.0}
dact = {k: [] for k in tilt_max}
prev = None
with torch.no_grad():
    for k in range(int(12.0 / env.step_dt)):
        t = k * env.step_dt
        cmd[:, 0] = vx_at(t)
        de.pin_commands(env, cmd)
        a = policy(obs)
        obs, _, dones, extras = wrapped.step(a)
        timeouts = extras.get("time_outs", torch.zeros_like(dones))
        f = (dones.bool() & ~timeouts.bool()).cpu().numpy()
        fell_at[f & ~fell] = t
        fell |= f
        phase = "stand" if t < 4 else "walk" if t < 11 else "down"
        g = robot.data.projected_gravity_b.cpu().numpy()
        tilt = np.degrees(np.arccos(np.clip(-g[:, 2], -1, 1)))[~fell]
        if len(tilt):
            tilt_max[phase] = max(tilt_max[phase], float(tilt.max()))
        if prev is not None:
            dact[phase].append(float((a - prev).abs().mean()))
        prev = a.clone()
print(f"RESULT vx {VX}: falls {int(fell.sum())}/{N} at {np.round(fell_at[fell], 1).tolist()}; max tilt "
      + ", ".join(f"{k} {v:.1f}" for k, v in tilt_max.items()) + "; action change "
      + ", ".join(f"{k} {np.mean(v):.3f}" for k, v in dact.items() if v))
