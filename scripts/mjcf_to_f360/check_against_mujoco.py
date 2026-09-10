#!/usr/bin/env python3
"""Cross-check the pure-Python MJCF reader against MuJoCo's own kinematics.

Needs the `mujoco` package, so run it from the RL project's environment:

    uv run --project ../microduck_rl python scripts/mjcf_to_f360/check_against_mujoco.py \
        ../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml

Compares every body's world position and rotation, and every joint's world
anchor and axis, at the zero configuration. With --bake DIR it also bakes the
meshes the way the Fusion script does and compares each baked vertex cloud
with MuJoCo's own (re-centred) mesh vertices under its adjusted geom pose.
That part needs scipy for the nearest-neighbour search.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mjcf_model as mm  # noqa: E402
import stl_tools  # noqa: E402


def check_meshes(m, model, bake_dir, groups=(0, 1, 2), tol=1e-6):
    import mujoco
    from scipy.spatial import cKDTree

    worst, gi, cache = 0.0, 0, {}
    for body in model.bodies:
        for geom in body.geoms:
            if geom.is_mesh and geom.group in groups and geom.mesh in model.meshes:
                mid = m.geom_dataid[gi]
                a = m.mesh_vertadr[mid]
                verts = m.mesh_vert[a:a + m.mesh_vertnum[mid]].astype(np.float64)
                rot = np.zeros(9)
                mujoco.mju_quat2Mat(rot, m.geom_quat[gi])
                expect = verts @ rot.reshape(3, 3).T + m.geom_pos[gi]
                baked_path = stl_tools.bake_geom(model, geom, bake_dir, 1.0, cache)
                baked = np.unique(np.array(stl_tools.read_stl(baked_path)).reshape(-1, 3), axis=0)
                err = max(cKDTree(baked).query(expect)[0].max(), cKDTree(expect).query(baked)[0].max())
                worst = max(worst, err)
                flag = "" if err < tol else "  <-- MISMATCH"
                print("mesh  %-22s in %-16s max vertex err %.2e%s" % (geom.mesh, body.name, err, flag))
            gi += 1
    return worst


def main(path, bake_dir=None, tol=1e-6):
    import mujoco

    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    model = mm.MjcfModel(path)

    worst = 0.0
    for body in model.articulated_bodies():
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body.name)
        if bid < 0:
            print("body %s not found in MuJoCo model" % body.name)
            worst = max(worst, 1.0)
            continue
        dp = np.abs(np.array(mm.mat4_pos(body.world)) - d.xpos[bid]).max()
        dr = np.abs(np.array(mm.mat4_rot(body.world)) - d.xmat[bid].reshape(3, 3)).max()
        worst = max(worst, dp, dr)
        flag = "" if max(dp, dr) < tol else "  <-- MISMATCH"
        print("body %-22s pos err %.2e rot err %.2e%s" % (body.name, dp, dr, flag))

    for j in model.joints:
        if j.jtype == "free":
            continue
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j.name)
        if jid < 0:
            print("joint %s not found in MuJoCo model" % j.name)
            worst = max(worst, 1.0)
            continue
        da = np.abs(np.array(j.world_pos()) - d.xanchor[jid]).max()
        dx = np.abs(np.array(j.world_axis()) - d.xaxis[jid]).max()
        rng = m.jnt_range[jid]
        drng = np.abs(np.array(j.range) - rng).max() if j.range is not None and m.jnt_limited[jid] else 0.0
        worst = max(worst, da, dx, drng)
        flag = "" if max(da, dx, drng) < tol else "  <-- MISMATCH"
        print("joint %-22s anchor err %.2e axis err %.2e range err %.2e%s" % (j.name, da, dx, drng, flag))

    if bake_dir:
        worst = max(worst, check_meshes(m, model, bake_dir, tol=tol))

    print("worst error %.3e -> %s" % (worst, "OK" if worst < tol else "FAIL"))
    return 0 if worst < tol else 1


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mjcf")
    ap.add_argument("--bake", metavar="DIR", help="also bake meshes into DIR and compare them with MuJoCo")
    args = ap.parse_args()
    sys.exit(main(args.mjcf, args.bake))
