#!/usr/bin/env python3
"""Inspect an MJCF model the way the Fusion script will see it.

Runs anywhere with Python 3 (no third-party packages). Prints the body tree
with world poses at the zero configuration, lists joints with world axes, and
can optionally bake the meshes into positioned STL copies plus an
assembly.json manifest. The baked output can also be used on its own: insert
each body's STLs into Fusion and add joints by hand from the listed axes.

    python dump_assembly.py path/to/robot.xml
    python dump_assembly.py path/to/robot.xml --bake out_dir --units mm --groups 0,1,2
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mjcf_model as mm  # noqa: E402
import stl_tools  # noqa: E402


def fmt(v, nd=4):
    return "[" + ", ".join("%.*f" % (nd, c) for c in v) + "]"


def print_tree(model, groups):
    for body in model.bodies:
        indent = "  " * body.depth
        pos = mm.mat4_pos(body.world)
        quat = mm.mat3_to_quat(mm.mat4_rot(body.world))
        n_mesh = len(body.mesh_geoms(groups))
        n_skip = len(body.geoms) - n_mesh
        print("%s%s  world pos %s quat %s  meshes %d (skipped geoms %d)"
              % (indent, body.name, fmt(pos), fmt(quat), n_mesh, n_skip))
        for j in body.joints:
            if j.jtype == "free":
                print("%s  freejoint %s" % (indent, j.name or ""))
                continue
            rng = " range %s" % fmt(j.range) if j.range else ""
            print("%s  joint %s (%s)  world origin %s axis %s%s"
                  % (indent, j.name, j.jtype, fmt(j.world_pos()), fmt(j.world_axis(), 3), rng))
        for i, g in enumerate(body.geoms):
            if not g.is_mesh:
                print("%s  - non-mesh geom %s (%s) skipped" % (indent, model.geom_label(g, i), g.gtype))
            elif g.group not in groups:
                print("%s  - %s group %d skipped" % (indent, model.geom_label(g, i), g.group))
            else:
                print("%s  - %s mesh %s pos %s" % (indent, model.geom_label(g, i), g.mesh, fmt(g.pos)))


def bake(model, out_dir, units, groups):
    unit_scale = stl_tools.UNIT_SCALES[units]
    cache = {}
    manifest = {"model": model.name, "source": model.path, "units": units, "bodies": [], "joints": []}
    for body in model.articulated_bodies():
        entry = {
            "name": body.name,
            "parent": body.parent.name,
            "world_pos": [c * unit_scale for c in mm.mat4_pos(body.world)],
            "world_quat_wxyz": mm.mat3_to_quat(mm.mat4_rot(body.world)),
            "world_matrix_row_major": mm.mat4_flat(body.world),
            "meshes": [],
        }
        for i, g in enumerate(body.mesh_geoms(groups)):
            if g.mesh not in model.meshes:
                continue
            path = stl_tools.bake_geom(model, g, out_dir, unit_scale, cache)
            entry["meshes"].append({"label": model.geom_label(g, i), "file": os.path.basename(path),
                                    "rgba": model.geom_rgba(g)})
            print("baked %s -> %s" % (g.mesh, os.path.basename(path)))
        manifest["bodies"].append(entry)
    for j in model.joints:
        if j.jtype == "free":
            continue
        manifest["joints"].append({
            "name": j.name, "type": j.jtype, "child": j.body.name, "parent": j.body.parent.name,
            "world_origin": [c * unit_scale for c in j.world_pos()],
            "world_axis": j.world_axis(),
            "range": j.range,
        })
    with open(os.path.join(out_dir, "assembly.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print("wrote %s" % os.path.join(out_dir, "assembly.json"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mjcf")
    ap.add_argument("--bake", metavar="OUT_DIR", help="write positioned STL copies and assembly.json here")
    ap.add_argument("--units", default="mm", choices=sorted(stl_tools.UNIT_SCALES), help="units of baked STLs")
    ap.add_argument("--groups", default="0,1,2", help="comma-separated geom groups to include")
    args = ap.parse_args()

    groups = {int(g) for g in args.groups.split(",") if g.strip() != ""}
    model = mm.MjcfModel(args.mjcf)
    print(model.summary())
    print_tree(model, groups)
    missing = [m.path for m in model.meshes.values() if not os.path.isfile(m.path)]
    for p in missing:
        print("MISSING mesh file: %s" % p)
    for w in model.warnings:
        print("warning: %s" % w)
    if args.bake:
        bake(model, args.bake, args.units, groups)
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
