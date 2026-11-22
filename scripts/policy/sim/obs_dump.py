"""Dump the walk policy's observations and actions in sim, standing and walking, for comparison
with a robotd recording (scripts/policy/obs_replay.py). Runs dukki_eval's setup.

    cd microduck_rl && uv run python ../dukki/scripts/policy/sim/obs_dump.py POLICY.onnx OUT.npz [--clean]
"""
import sys
sys.path.insert(0, "scripts")
import numpy as np
import torch
import dukki_eval as de

de.SCENARIOS = {"stand": (0.0, 0.0, 0.0), "walk_0.15": (0.15, 0.0, 0.0)}
policy_path, out = sys.argv[1], sys.argv[2]
clean = "--clean" in sys.argv
cfg_loader = de.load_env_cfg
def load_env_cfg(task):
    cfg = cfg_loader(task)
    if clean:
        cfg.observations["actor"].enable_corruption = False
    return cfg
de.load_env_cfg = load_env_cfg
env, wrapped, runner, cmd = de.build("Mjlab-Velocity-Flat-Dukki", 8, 12.0)
policy = de.OnnxPolicy(policy_path)
wrapped.reset(); env.reset(seed=0); de.pin_commands(env, cmd)
obs = wrapped.get_observations()
robot = env.scene["robot"]; act = robot.actuators[0]
O, A, Q, T, G = [], [], [], [], []
with torch.no_grad():
    for k in range(600):
        de.pin_commands(env, cmd)
        a = policy(obs)
        O.append(obs["actor"].cpu().numpy().copy()); A.append(a.cpu().numpy().copy())
        Q.append(robot.data.joint_pos[:, act._target_ids_list].cpu().numpy().copy())
        T.append(act._processed_actions.cpu().numpy().copy() if hasattr(act, "_processed_actions") else np.zeros(1))
        G.append(robot.data.root_link_ang_vel_b.cpu().numpy().copy())
        obs, _, dones, _ = wrapped.step(a)
np.savez(out, obs=np.stack(O), act=np.stack(A), q=np.stack(Q), gyro_true=np.stack(G),
         scenario=np.array([s for s in de.SCENARIOS for _ in range(8)]), names=np.array(list(act.target_names)))
print("wrote", out, np.stack(O).shape)
