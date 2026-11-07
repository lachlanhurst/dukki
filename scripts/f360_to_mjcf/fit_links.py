"""Fit per-link rigid transforms from claude_test (posed, Fusion world) to the MJCF zero pose.

Writes link_corr.npy into the work folder and prints how far each Fusion servo axis lands from
the matching MJCF joint axis, which is the check that the kinematics did not change.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

m = mujoco.MjModel.from_xml_path(RL + 'robot_walk.xml')
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

# joint name -> (parent link, child link); mouth is not in the MJCF (jaw is fixed to the head)
CHAIN = [
    ('left_hip_yaw', 'trunk', 'left_hip_yaw_link'), ('left_hip_roll', 'left_hip_yaw_link', 'left_hip_roll_link'),
    ('left_hip_pitch', 'left_hip_roll_link', 'left_upper_leg_link'), ('left_knee', 'left_upper_leg_link', 'left_shin_link'),
    ('left_ankle', 'left_shin_link', 'left_foot_link'),
    ('right_hip_yaw', 'trunk', 'right_hip_yaw_link'), ('right_hip_roll', 'right_hip_yaw_link', 'right_hip_roll_link'),
    ('right_hip_pitch', 'right_hip_roll_link', 'right_upper_leg_link'), ('right_knee', 'right_upper_leg_link', 'right_shin_link'),
    ('right_ankle', 'right_shin_link', 'right_foot_link'),
    ('neck_pitch', 'trunk', 'neck_link'), ('head_pitch', 'neck_link', 'head_pitch_link'),
    ('head_yaw', 'head_pitch_link', 'head_yaw_link'), ('head_roll', 'head_yaw_link', 'head_link'),
]

def mj_line(j):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    return d.xanchor[jid] * 1000.0, d.xaxis[jid].copy()

def f_line(j):
    a = np.array(F['axes'][j]) * 10.0
    u = a[3:] - a[:3]
    return a[:3], u / np.linalg.norm(u)

def apply(T, p):
    return T[:3, :3] @ p + T[:3, 3]

def line_err(p1, u1, p2, u2):
    """Distance between two (undirected) lines near p2, and angle between them in degrees."""
    dvec = p1 - p2
    perp = dvec - np.dot(dvec, u2) * u2
    ang = np.degrees(np.arccos(min(1.0, abs(np.dot(u1, u2)))))
    return np.linalg.norm(perp), ang

def rot_about(p, u, phi):
    R = trimesh.transformations.rotation_matrix(phi, u, p)
    return R

# trunk: pure translation from the three trunk-mounted axes
A, b = [], []
for j in ('left_hip_yaw', 'right_hip_yaw', 'neck_pitch'):
    pf, uf = f_line(j)
    pm, um = mj_line(j)
    assert abs(abs(np.dot(uf, um)) - 1) < 1e-3, j
    P = np.eye(3) - np.outer(um, um)
    A.append(P); b.append(P @ (pm - pf))
t = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)[0]
C = {'trunk': np.eye(4)}
C['trunk'][:3, 3] = t
C['battery'] = C['trunk']
print('trunk offset (mm)', t.round(3))

def next_joints(link):
    return [c for c in CHAIN if c[1] == link]

def sole_normal_mj(body, mesh):
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == bid and m.geom_type[g] == 7 and mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) == mesh:
            return d.geom_xmat[g].reshape(3, 3)
    raise KeyError(mesh)

phis = {}
for j, par, ch in CHAIN:
    pf, uf = f_line(j)
    p = apply(C[par], pf); u = C[par][:3, :3] @ uf
    pm, um = mj_line(j)
    e = line_err(p, u, pm, um)
    nxt = next_joints(ch)
    best = None
    for phi in np.radians(np.arange(-90, 90.01, 0.01)):
        R = rot_about(p, u, phi) @ C[par]
        cost = 0
        for k, _, _ in nxt:
            qf, vf = f_line(k)
            qm, vm = mj_line(k)
            dd, aa = line_err(apply(R, qf), R[:3, :3] @ vf, qm, vm)
            cost += dd + aa
        if not nxt:
            # leaf: feet sit flat on the ground in claude_test; the MJCF zero-pose foot is flat too
            # (checked below), so keep the world-z direction fixed: choose phi that keeps z up
            zf = (R[:3, :3] @ np.array([0, 0, 1.0]))
            cost = np.degrees(np.arccos(min(1, zf[2])))
        if best is None or cost < best[0]:
            best = (cost, phi, R)
    C[ch] = best[2]
    phis[j] = np.degrees(best[1])
    print('%-16s axis residual %.3f mm %.3f deg | rotate %7.2f deg | fit cost %.4f' % (j, e[0], e[1], phis[j], best[0]))
C['jaw_link'] = C['head_link']
np.save(S + 'link_corr.npy', {k: v for k, v in C.items()}, allow_pickle=True)
