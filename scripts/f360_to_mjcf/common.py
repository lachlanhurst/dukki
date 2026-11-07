"""Shared paths and helpers for the Fusion (claude_test) to MJCF conversion."""
import json
import numpy as np
import mujoco
import trimesh

import os

_HERE = os.path.dirname(os.path.abspath(__file__))
# the microduck robot folder in microduck_rl, and the work folder fusion_export.py wrote into
RL = os.environ.get('MICRODUCK_RL_ROBOT', os.path.join(
    _HERE, '..', '..', '..', 'microduck_rl', 'src', 'mjlab_microduck', 'robot', 'microduck')) + '/'
S = os.environ.get('F360_MJCF_WORK', os.path.join(_HERE, 'work')) + '/'
F = json.load(open(S + 'fusion_data.json'))
NAMES = json.load(open(os.path.join(_HERE, 'names.json')))


def link_corrections():
    return np.load(S + 'link_corr.npy', allow_pickle=True).item()


C = link_corrections() if os.path.exists(S + 'link_corr.npy') else None
LINK_BODY = {
    'trunk': 'trunk_base', 'battery': 'trunk_base',
    'left_hip_yaw_link': 'yaw2roll', 'left_hip_roll_link': 'hip_l', 'left_upper_leg_link': 'upper_leg_left',
    'left_shin_link': 'leg', 'left_foot_link': 'ankle_left',
    'right_hip_yaw_link': 'bearing_roll', 'right_hip_roll_link': 'hip_l_2', 'right_upper_leg_link': 'upper_leg_right',
    'right_shin_link': 'leg_2', 'right_foot_link': 'ankle_right',
    'neck_link': 'neck', 'head_pitch_link': 'neck_pitch', 'head_yaw_link': 'yaw_roll_motion',
    'head_link': 'jaw_soft', 'jaw_link': 'jaw_soft',
}

def fusion_T(arr):
    """Fusion transform2 array (row-major 4x4, cm) -> 4x4 in mm."""
    T = np.array(arr, dtype=float).reshape(4, 4)
    T[:3, 3] *= 10.0
    return T

def units():
    """(link, comp, mesh name, world-MJCF 4x4 in mm) for every part, one per occurrence."""
    out, seen = [], set()
    for g, us in F['links'].items():
        link = g[:-len('_rigid')]
        for u in us:
            if u['comp'] == 'trunk_shell_left' or u['path'] in seen:
                continue
            if link == 'battery' and u['comp'] != 'battery 6s lipo':
                continue
            seen.add(u['path'])
            out.append((link, u['comp'], NAMES[u['comp']], C[link] @ fusion_T(u['T'])))
    return out

def body_poses(model_path):
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    X = {}
    for b in range(1, m.nbody):
        T = np.eye(4)
        T[:3, :3] = d.xmat[b].reshape(3, 3)
        T[:3, 3] = d.xpos[b] * 1000.0
        X[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)] = T
    return m, d, X
