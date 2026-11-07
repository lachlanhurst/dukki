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

def underside_normal(V, faces, down):
    """Outward normal of the largest flat face facing `down`: the Onshape sole's underside."""
    t = trimesh.Trimesh(V, faces, process=False)
    n, a, c = t.face_normals, t.area_faces, t.triangles_center
    sel = n @ down > 0.98
    planes = {}
    for off, ar, nn in zip(np.round(c[sel] @ down / 0.3), a[sel], n[sel]):
        s = planes.setdefault(off, [0.0, np.zeros(3)])
        s[0] += ar
        s[1] += ar * nn
    nn = max(planes.values(), key=lambda s: s[0])[1]
    return nn / np.linalg.norm(nn)

def profile_slope(V, down, along, bin_mm=4.0):
    """Heel-to-toe slope (degrees) of a line through the lowest point of each `bin_mm` slice of the
    underside, relative to the plane normal to `down`. The first and last slices (the edge fillets)
    are left out. The Fusion sole's underside is not one plane but a very shallow curve made of
    facets, so this levels its chord rather than any one facet."""
    depth, s = V @ down, V @ along
    keep = depth > depth.max() - 1.5
    depth, s = depth[keep], s[keep]
    edges = np.arange(s.min(), s.max() + bin_mm, bin_mm)
    mid, low = [], []
    for a in edges[1:-2]:
        inb = (s >= a) & (s < a + bin_mm)
        if inb.any():
            mid.append(a + bin_mm / 2)
            low.append(depth[inb].max())
    return np.degrees(np.arctan(np.polyfit(mid, low, 1)[0]))

def sole_normal_mj(geom_name):
    """Underside normal of the Onshape sole collision mesh, MJCF world at the zero pose."""
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    mid = m.geom_dataid[g]
    V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
    faces = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
    return underside_normal(V @ d.geom_xmat[g].reshape(3, 3).T, faces, np.array([0, 0, -1.0]))

def sole_verts_f360(link, mesh):
    """Vertices of the Fusion sole in the claude_test world (mm)."""
    for u in F['links'][link + '_rigid']:
        if NAMES[u['comp']] == mesh:
            t = trimesh.load(S + 'raw/%s.stl' % mesh)
            t.apply_transform(fusion_T(u['T']))
            return t.vertices
    raise KeyError(mesh)

# feet: the new sole's underside is levelled against the Onshape sole's plane, which sits flat at
# STAND. claude_test's feet are about 1.4 degrees off flat about the ankle axis, so its world z is
# no guide.
SOLES = {'left_foot_link': ('sole_left', 'left_foot_collision'), 'right_foot_link': ('sole_right', 'right_foot_collision')}

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
            # head and feet: keep claude_test's world z up (the feet are refined below)
            zf = (R[:3, :3] @ np.array([0, 0, 1.0]))
            cost = np.degrees(np.arccos(min(1, zf[2])))
        if best is None or cost < best[0]:
            best = (cost, phi, R)
    if ch in SOLES:
        n_mj = sole_normal_mj(SOLES[ch][1])
        along = np.cross(um, n_mj)
        along /= np.linalg.norm(along)
        V_f = sole_verts_f360(ch, SOLES[ch][0])
        phi0, best = best[1], None
        for phi in phi0 + np.radians(np.arange(-4, 4.001, 0.01)):
            R = rot_about(p, u, phi) @ C[par]
            cost = abs(profile_slope(V_f @ R[:3, :3].T + R[:3, 3], n_mj, along))
            if best is None or cost < best[0]:
                best = (cost, phi, R)
        print('%-16s sole levelled: %.2f deg from claude_test\'s world z' % (j, np.degrees(best[1] - phi0)))
    C[ch] = best[2]
    phis[j] = np.degrees(best[1])
    print('%-16s axis residual %.3f mm %.3f deg | rotate %7.2f deg | fit cost %.4f' % (j, e[0], e[1], phis[j], best[0]))
C['jaw_link'] = C['head_link']
np.save(S + 'link_corr.npy', {k: v for k, v in C.items()}, allow_pickle=True)
