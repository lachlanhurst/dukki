"""Read, transform and write STL files with the standard library only.

Used to "bake" each MJCF geom's local pose (and mesh scale) into a copy of
its STL so Fusion can import the copy straight into the body's component
without needing to move a mesh body afterwards.
"""

import hashlib
import os
import struct

from mjcf_model import mat3_det, mat4_apply_point, mat4_from, mat4_rot, vec_cross, vec_normalise, mat4_flat


def read_stl(path):
    """Return a list of triangles, each a tuple of three (x, y, z) vertex tuples."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            return _read_binary(data, count)
    if data[:5].lower() == b"solid":
        return _read_ascii(data.decode("utf-8", errors="replace"))
    raise ValueError("unrecognised STL file: %s" % path)


def _read_binary(data, count):
    tris = []
    for rec in struct.iter_unpack("<12fH", data[84:84 + 50 * count]):
        tris.append(((rec[3], rec[4], rec[5]), (rec[6], rec[7], rec[8]), (rec[9], rec[10], rec[11])))
    return tris


def _read_ascii(text):
    tris, verts = [], []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(verts) == 3:
                tris.append(tuple(verts))
                verts = []
    return tris


def _normal(a, b, c):
    n = vec_cross([b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)])
    try:
        return vec_normalise(n)
    except ValueError:
        return [0.0, 0.0, 0.0]


def write_binary_stl(path, tris, header="baked by mjcf_to_f360"):
    hdr = header.encode("ascii", errors="replace")[:80].ljust(80, b"\0")
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = _normal(a, b, c)
            f.write(struct.pack("<12fH", n[0], n[1], n[2], *a, *b, *c, 0))


def transform_triangles(tris, mat4, scale=(1.0, 1.0, 1.0), unit_scale=1.0):
    """Apply mesh scale, then the rigid transform, then a unit conversion factor."""
    flip = (scale[0] * scale[1] * scale[2]) < 0 or mat3_det(mat4_rot(mat4)) < 0
    out = []
    for tri in tris:
        pts = []
        for v in tri:
            p = mat4_apply_point(mat4, [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
            pts.append((p[0] * unit_scale, p[1] * unit_scale, p[2] * unit_scale))
        if flip:
            pts = [pts[0], pts[2], pts[1]]
        out.append(tuple(pts))
    return out


def baked_name(mesh, geom, unit_scale):
    key = repr((mesh.name, [round(v, 9) for v in mat4_flat(mat4_from(geom.rot, geom.pos))],
                [round(s, 9) for s in mesh.scale], round(unit_scale, 9)))
    digest = hashlib.md5(key.encode("utf-8")).hexdigest()[:10]
    return "%s__%s.stl" % (mesh.name, digest)


def bake_geom(model, geom, out_dir, unit_scale, cache=None, overwrite=False):
    """Write a copy of the geom's mesh with its body-relative pose baked in.

    Returns the output path. Identical (mesh, pose, scale) combinations share
    one file. `cache` is an optional dict for reuse within a run.
    """
    mesh = model.meshes[geom.mesh]
    out_path = os.path.join(out_dir, baked_name(mesh, geom, unit_scale))
    if cache is not None and out_path in cache:
        return out_path
    if overwrite or not os.path.isfile(out_path):
        tris = read_stl(mesh.path)
        tris = transform_triangles(tris, mat4_from(geom.rot, geom.pos), mesh.scale, unit_scale)
        os.makedirs(out_dir, exist_ok=True)
        write_binary_stl(out_path, tris, "mjcf_to_f360 %s in %s" % (mesh.name, geom.body.name))
    if cache is not None:
        cache[out_path] = True
    return out_path


UNIT_SCALES = {"m": 1.0, "cm": 100.0, "mm": 1000.0}
