"""Compare every scene against the Onshape-era model: mass, sole height at STAND, self-contacts.

OLD is a copy of the robot folder from before the conversion (see README.md).
"""
import glob
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RL as NEW, S  # noqa: E402

OLD = S + 'old_model/'

def info(m, d):
    return m.body_subtreemass[1]

def self_contacts(m, d):
    out = []
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if b1 == 0 or b2 == 0:
            continue
        n = lambda g: '%s/%s' % (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) if m.geom_dataid[g] >= 0 else m.geom_type[g])
        out.append('%s <-> %s (%.1f mm)' % (n(c.geom1), n(c.geom2), c.dist * 1000))
    return out

def foot_bottom(m, d):
    zs = []
    for name in ('left_foot_collision', 'right_foot_collision'):
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
        if g < 0:
            return None
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        zs.append(w[:, 2].min())
    return zs

for scene in sorted(glob.glob(NEW + 'scene*.xml')):
    b = os.path.basename(scene)
    res = {}
    for tag, base in (('old', OLD), ('new', NEW)):
        m = mujoco.MjModel.from_xml_path(base + b)
        d = mujoco.MjData(m)
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'STAND')
        if k >= 0:
            mujoco.mj_resetDataKeyframe(m, d, k)
        mujoco.mj_forward(m, d)
        fb = foot_bottom(m, d)
        res[tag] = (m.body_subtreemass[1], fb, self_contacts(m, d), m.ngeom)
    o, n = res['old'], res['new']
    print('%-28s mass %.4f -> %.4f  geoms %d -> %d' % (b, o[0], n[0], o[3], n[3]))
    if o[1]:
        print('    STAND sole bottom z (mm): old %s new %s' % (np.round(np.array(o[1]) * 1000, 2), np.round(np.array(n[1]) * 1000, 2)))
    print('    self contacts at STAND: old %d new %d' % (len(o[2]), len(n[2])))
    for c in n[2][:12]:
        print('      new:', c)
