"""Which geoms of the stand-up model collide, and which visual parts sink into the floor in the robot's stuck pose."""
import json, math, sys
import numpy as np, torch, mujoco
import importlib.util
import os
spec = importlib.util.spec_from_file_location("sfr", os.path.join(os.path.dirname(os.path.abspath(__file__)), "standup_from_robot.py"))
sfr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sfr)
from mjlab_microduck.tasks.mdp import _env_origin_z, _servo_joint_ids

task = "Mjlab-StandUp-Flat-Backlash-Dukki-Distill"
env = sfr.build(task, 8, 5.0, "cuda:0")
env.reset(seed=0)
m = env.sim.mj_model
robot = env.scene["robot"]
print("== collision-enabled geoms (contype|conaffinity != 0)")
for g in range(m.ngeom):
    if m.geom_contype[g] or m.geom_conaffinity[g]:
        b = m.geom_bodyid[g]
        print(f"  {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '-':28s} body {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b):24s} type {m.geom_type[g]} contype {m.geom_contype[g]} conaff {m.geom_conaffinity[g]} group {m.geom_group[g]}")

def lowest(d):
    """Per body, the lowest point of its visual mesh geoms (world z, m)."""
    out = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        b = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        out[b] = min(out.get(b, 9), float(w[:, 2].min()))
    return out

pose = json.load(open(sys.argv[1]))
n, dev = 8, env.device
servo = _servo_joint_ids(env, robot)
default = robot.data.default_joint_pos[0, servo]
ids = torch.arange(n, device=dev)
g = torch.tensor(pose["gravity"], device=dev); q = default + torch.tensor(pose["q_minus_default"], device=dev)
act = torch.tensor(pose["action"], device=dev).expand(n, -1).contiguous()
d = env.sim.data
d.qpos[ids, 2] = 0.08; d.qpos[ids, 3:7] = sfr.quat_from_gravity(g, torch.zeros(n, device=dev)); d.qvel[ids] = 0
hinge = 7 + torch.arange(d.qpos.shape[1] - 7, device=dev); d.qpos[ids.unsqueeze(1), hinge.unsqueeze(0)] = 0
for j, s in enumerate(servo): d.qpos[ids, 7 + s] = q[j]
for pin in (True, False):
    quat = d.qpos[ids, 3:7].clone()
    for _ in range(75):
        env.step(act)
        if pin: d.qpos[ids, 3:7] = quat; d.qvel[ids, 3:6] = 0
    cd = mujoco.MjData(m); cd.qpos[:] = d.qpos[0].cpu().numpy(); mujoco.mj_forward(m, cd)
    print(f"== {'pinned at the robot tilt' if pin else 'released'}: trunk z {cd.qpos[2]*1000:.0f} mm, gravity {robot.data.projected_gravity_b[0].cpu().numpy().round(2)}")
    print("   visual parts lowest point (mm), below -3 mm marked:")
    for b, z in sorted(lowest(cd).items(), key=lambda x: x[1])[:14]:
        print(f"     {b:26s} {z*1000:+6.1f} {'<-- in the floor' if z < -0.003 else ''}")
    print("   contacts:", sorted({mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1]) for c in cd.contact[:cd.ncon]} | {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2]) for c in cd.contact[:cd.ncon]}))
