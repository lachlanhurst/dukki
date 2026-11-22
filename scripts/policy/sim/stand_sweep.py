"""Which sim change makes the walk policy's stand shake like the real robot's (22/11/2026)?
One variant per process: cd microduck_rl && uv run python ../dukki/scripts/policy/sim/stand_sweep.py POLICY.onnx VARIANT
Robot reference (robotd, 9250, legs 0.7/head 0.5): stand |d action| 0.090/tick, joint pos std 0.03-0.15 rad,
gyro std 0.5-1.1 rad/s; walk 0.15 |d action| 0.171.

Variants: base; actdelay_LO_HI (servo command delay, 5 ms physics steps); obsdelay1 (every sensor one
tick late); imudelayN (gyro and gravity N ticks late); noiseK (sensor noise times K); footF (foot
friction F); encD (encoder offsets up to +-D degrees); combo; backlash (the backlash task). kp, kd and
joint friction go through the task's own DUKKI_KP, DUKKI_KD and DUKKI_FRICTION.
Results in docs/policy-testing.md."""
import sys, json
sys.path.insert(0, "scripts")
import numpy as np
import torch
import dukki_eval as de

policy_path, variant = sys.argv[1], sys.argv[2]
task = "Mjlab-Velocity-Flat-Backlash-Dukki" if variant == "backlash" else "Mjlab-Velocity-Flat-Dukki"
de.SCENARIOS = {"stand": (0.0, 0.0, 0.0), "walk_0.15": (0.15, 0.0, 0.0)}
loader = de.load_env_cfg


def load_env_cfg(t):
    cfg = loader(t)
    acts = cfg.scene.entities["robot"].articulation.actuators
    obs = cfg.observations["actor"].terms
    if variant.startswith("actdelay"):  # actdelay_3_4: physics steps of 5 ms
        lo, hi = map(int, variant.split("_")[1:])
        for a in acts:
            a.delay_min_lag, a.delay_max_lag = lo, hi
    if variant == "obsdelay1":
        for n in ("base_ang_vel", "projected_gravity", "joint_pos", "joint_vel"):
            obs[n].delay_min_lag = obs[n].delay_max_lag = 1
            obs[n].delay_update_period = 0
    if variant.startswith("imudelay"):  # imudelay2: gyro and gravity both N policy ticks late
        n = int(variant[8:])
        for name in ("base_ang_vel", "projected_gravity"):
            obs[name].delay_min_lag = obs[name].delay_max_lag = n
            obs[name].delay_update_period = 0
    if variant.startswith("noise"):
        k = float(variant[5:])
        for n in ("base_ang_vel", "joint_vel", "projected_gravity", "joint_pos"):
            obs[n].noise.n_min *= k
            obs[n].noise.n_max *= k
    if variant.startswith("foot"):
        f = float(variant[4:])
        cfg.events["foot_friction"].params["ranges"] = (f, f)
    if variant.startswith("enc") or variant == "combo":
        d = 3.0 if variant == "combo" else float(variant[3:])
        r = d * 3.14159265 / 180
        cfg.events["encoder_bias"].params["bias_range"] = (-r, r)
    if variant == "combo":
        for a in acts:
            a.delay_min_lag, a.delay_max_lag = 3, 4
        for n in ("base_ang_vel", "joint_vel", "projected_gravity", "joint_pos"):
            obs[n].noise.n_min *= 2
            obs[n].noise.n_max *= 2
        cfg.events["foot_friction"].params["ranges"] = (0.5, 0.7)
    return cfg


de.load_env_cfg = load_env_cfg
env, wrapped, runner, cmd = de.build(task, 8, 12.0)
policy = de.OnnxPolicy(policy_path)
wrapped.reset(); env.reset(seed=0); de.pin_commands(env, cmd)
obs = wrapped.get_observations()
robot = env.scene["robot"]
O, A, F = [], [], np.zeros(16)
with torch.no_grad():
    for k in range(600):
        de.pin_commands(env, cmd)
        a = policy(obs)
        O.append(obs["actor"].cpu().numpy().copy()); A.append(a.cpu().numpy().copy())
        obs, _, dones, extras = wrapped.step(a)
        to = extras.get("time_outs", torch.zeros_like(dones))
        if k >= 100:
            F += (dones.bool() & ~to.bool()).cpu().numpy()
O, A = np.stack(O)[100:], np.stack(A)[100:]
res = {}
for i, sc in enumerate(de.SCENARIOS):
    sl = slice(8 * i, 8 * i + 8)
    o, a = O[:, sl], A[:, sl]
    res[sc] = {"falls": int(F[sl].sum()), "d_action": float(np.abs(np.diff(a, axis=0)).mean()),
               "q_std": float(o[:, :, 6:20].std(0).mean()), "gyro_std": float(o[:, :, 0:3].std(0).mean()),
               "qd_std": float(o[:, :, 20:34].std(0).mean())}
print("RESULT", variant, json.dumps(res))
