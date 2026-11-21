"""Calibration jig: a two-piece cradle the assembled robot lies in on its back.

The robot is laid in with every joint at zero (the MJCF's reference pose, legs straight,
head level) and the servos limp. Each link settles into its own pocket, so the output
encoder of every servo can be read against a known joint angle and the zero offsets
worked out. The battery is not fitted.

How the cradle is made:

    1. Pose the MJCF at q = 0 and turn it onto its back: jig X runs from the feet to the
       head, Y across the robot, Z up. The front of the robot faces up.
    2. Ray-cast the visual meshes from below on a fine grid. That gives the height of the
       robot's underside above the bed at every point.
    3. Condition that height map so the jig only touches what it should:
         - the battery is left out, and the jig stays out of the bay it leaves: under
           the bridge PCB and the battery's footprint, plus a margin, there is only a
           `bay_tie` high grid (3 mm), far below the board and its wiring;
         - meshes that move with an uncalibrated joint (the jaw, on the mouth servo) get
           extra clearance;
         - narrow upward slots (vents, gaps between parts) are filled in by a grey opening,
           so ribs never poke into them;
         - a spherical erosion keeps every jig surface `clearance` away from the robot in
           every direction, not only vertically.
    4. Fill the space under that surface with an open egg-crate of thin vertical ribs,
       not a solid block, and only where it does something. Under the robot a rib rises
       to the shell and carries it. Where a rib meets the robot's outline it carries on
       as a locating fin, `fin` long, up to `top` above the lowest point (`top_head`
       under the head, which sits deeper), so the sides of each link are held. Anywhere
       else near the robot, and across the gap between the legs, a rib is only a low
       tie, `frame` high (`dovetail_height` in the head section, from the joints up to
       the head), that keeps the lattice in one piece. Away from the robot there is
       nothing, apart from the walls along the split, which carry the dovetails. The ribs
       are solid. Nothing overhangs, so the jig prints without supports, and the ribs are
       a whole number of extrusion widths so a slicer prints them as perimeters only.
       Any raised part of a rib shorter than `min_feature` along the rib is cut down, so
       there are no thin free-standing spikes for the printer to knock over.
    5. Split it in two across the gap under the neck, so the trunk lies wholly in one
       section and the head wholly in the other. Two vertical dovetails join the
       sections, the head section's tabs dropping into sockets in the body section with
       `fit` clearance all round. The joint blocks are only `dovetail_height` tall.

Outputs, in --out:

    jig_body.stl, jig_head.stl     the two sections, in mm, base on the bed
    jig_preview.glb, .txt          robot and both sections in jig coordinates (with --preview)
    jig_report.json                sizes, volumes, contact per link, the parameters used

Run from the repository root:

    uv run --with mujoco --with manifold3d --with trimesh --with shapely --with scipy \\
        --with numpy --with fast-simplification python scripts/calib_jig/make_jig.py \\
        --mjcf ../microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml \\
        --out hardware/cad/calib_jig --preview
"""

from __future__ import annotations

import argparse
import json
from functools import reduce
from pathlib import Path
from types import SimpleNamespace

import manifold3d as m3
import mujoco
import numpy as np
import trimesh
from scipy import ndimage
from shapely.geometry import LineString, Polygon
from shapely.geometry.polygon import orient

# Jig frame from the MJCF world frame: X = robot up (feet to head), Y = robot right,
# Z = robot forward (the robot lies on its back). Proper rotation, det +1.
R_JIG = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])

VISUAL_GROUP = 2
HIDDEN_GROUP = 5
BIG = 1000.0  # height where no robot is above, mm


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--preview", action="store_true", help="also write jig_preview.glb")
    p.add_argument("--res", type=float, default=0.25, help="height map grid, mm")
    p.add_argument("--clearance", type=float, default=0.4, help="gap between jig and shells, mm")
    p.add_argument("--slot", type=float, default=6.0, help="fill upward slots narrower than this, mm")
    p.add_argument("--bay-margin", type=float, default=3.0, help="keep-out around the bridge PCB bay, mm")
    p.add_argument("--bay-tie", type=float, default=3.0, help="tie height under the bridge PCB bay, mm")
    p.add_argument("--relieve", default="jaw:2.0", help="mesh:extra_mm,... given extra clearance")
    p.add_argument("--rib", type=float, default=1.2, help="rib thickness, mm (3 x 0.4 mm lines)")
    p.add_argument("--pitch", type=float, default=14.0, help="rib spacing, mm")
    p.add_argument("--top", type=float, default=35.0, help="cradle depth above the lowest point, mm")
    p.add_argument("--top-head", type=float, default=45.0, help="cradle depth in the head section, mm")
    p.add_argument("--min-feature", type=float, default=7.0, help="shortest raised rib length, mm")
    p.add_argument("--dovetail-height", type=float, default=10.0, help="joint block height, mm")
    p.add_argument("--base-gap", type=float, default=3.0, help="lowest point of the robot above the bed, mm")
    p.add_argument("--margin", type=float, default=6.0, help="ties this far around the robot, mm")
    p.add_argument("--frame", type=float, default=4.0, help="height of the ties between ribs, mm")
    p.add_argument("--fin", type=float, default=8.0, help="locating fin length beyond the robot, mm")
    p.add_argument("--spine", type=float, default=2.0, help="wall thickness each side of the split, mm")
    p.add_argument("--fit", type=float, default=0.15, help="dovetail clearance, mm")
    p.add_argument("--bed", type=float, default=190.0, help="printer bed, mm square")
    return p.parse_args()


# ---------------------------------------------------------------------------------------
# Robot


def load_robot(mjcf: Path):
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)  # every joint at 0
    mujoco.mj_kinematics(model, data)
    return model, data


def mesh_name(model, g: int) -> str:
    return model.mesh(model.geom_dataid[g]).name


def visual_geoms(model) -> list[int]:
    return [
        g
        for g in range(model.ngeom)
        if model.geom_group[g] == VISUAL_GROUP and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
    ]


def geom_mesh_jig(model, data, g: int) -> tuple[np.ndarray, np.ndarray]:
    """Vertices (mm, jig frame, before the Z shift) and faces of one mesh geom."""
    mid = model.geom_dataid[g]
    va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
    v = model.mesh_vert[va : va + vn] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
    return (v @ R_JIG.T) * 1000.0, model.mesh_face[fa : fa + fn].copy()


# ---------------------------------------------------------------------------------------
# Height map


def height_map(model, data, xs, ys, z0_mm, battery: set[int]):
    """Lowest robot surface above each grid point (mm, jig frame), with and without the
    battery, and the geom hit without it."""
    rt = R_JIG.T
    up = rt @ np.array([0.0, 0.0, 1.0])
    mask = np.zeros(6, np.uint8)
    mask[VISUAL_GROUP] = 1
    gid = np.zeros(1, np.int32)
    start = (z0_mm - 10.0) / 1000.0
    h_with = np.full((len(xs), len(ys)), BIG)
    h_without = np.full_like(h_with, BIG)
    hit = np.full(h_with.shape, -1, int)
    retry = []

    def cast(x, y):
        dist = mujoco.mj_ray(model, data, rt @ np.array([x / 1000, y / 1000, start]), up, mask, 1, -1, gid)
        return (start + dist) * 1000.0 if dist >= 0 else BIG, int(gid[0]) if dist >= 0 else -1

    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            z, g = cast(x, y)
            h_with[i, j] = h_without[i, j] = z
            hit[i, j] = g
            if g in battery:
                retry.append((i, j))

    saved = model.geom_group.copy()
    for g in battery:
        model.geom_group[g] = HIDDEN_GROUP
    for i, j in retry:
        h_without[i, j], hit[i, j] = cast(xs[i], ys[j])
    model.geom_group[:] = saved
    return h_with, h_without, hit


def disk(radius_px: float) -> np.ndarray:
    r = int(np.ceil(radius_px))
    yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
    return (xx**2 + yy**2) <= radius_px**2 + 1e-9


def allowed_height(args, h_with, h_without, hit, model, battery, ztop_rows):
    """Highest the jig may reach at each grid point without touching the robot, mm above
    the bed (capped at the cradle depth), the band around the robot, its footprint, the
    part hit at each point, and the bay the jig must stay out of."""
    res = args.res
    h = h_without.copy()

    # Parts that move with an uncalibrated joint get extra clearance.
    relief = {}
    for item in filter(None, args.relieve.split(",")):
        name, mm = item.split(":")
        relief[name.strip()] = float(mm)
    names = np.array(
        [mesh_name(model, g) if g >= 0 and model.geom_dataid[g] >= 0 else "" for g in range(-1, model.ngeom)]
    )
    hit_names = names[hit + 1]
    for name, mm in relief.items():
        h[hit_names == name] -= mm

    # Fill upward slots narrower than --slot. Grey opening never raises the surface.
    h = ndimage.grey_opening(h, footprint=disk(args.slot / 2 / res))

    # Keep `clearance` from the robot in every direction: erosion by a ball. A half
    # cell is added for the grid.
    r = args.clearance + 0.5 * res
    fp = disk(r / res)
    yy, xx = np.mgrid[: fp.shape[0], : fp.shape[1]] - fp.shape[0] // 2
    ball = np.sqrt(np.maximum(r**2 - (xx * res) ** 2 - (yy * res) ** 2, 0.0))
    h = ndimage.grey_erosion(h, footprint=fp, structure=np.where(fp, ball, 0.0))

    footprint = h_without < BIG / 2
    band = ndimage.binary_dilation(footprint, structure=disk(args.margin / res))
    clear = np.minimum(h, ztop_rows[:, None])

    # The bay the battery leaves: the bridge PCB and its wiring, and the battery's own
    # footprint. Close the gaps between the parts seen in it, fill it and add the margin.
    # Only a low tie stands there (see `profile`).
    bay = np.char.startswith(hit_names.astype(str), "bridge") | (h_with < h_without - 0.01)
    bay = ndimage.binary_closing(bay, structure=disk(4.0 / res))
    bay = ndimage.binary_fill_holes(bay)
    bay = ndimage.binary_dilation(bay, structure=disk(args.bay_margin / res))
    clear_raw = clear.copy()
    clear[bay] = 0.0
    return clear, clear_raw, band, footprint, hit_names, bay


# ---------------------------------------------------------------------------------------
# Solids


def cross_section(points: np.ndarray) -> m3.CrossSection:
    """A simple closed outline, either winding, as a manifold cross section."""
    pts = np.asarray(points, float)
    keep = np.r_[True, np.any(np.abs(np.diff(pts, axis=0)) > 1e-9, axis=1)]
    pts = pts[keep]
    x, y = pts[:, 0], pts[:, 1]
    if np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y) < 0:
        pts = pts[::-1]
    return m3.CrossSection([pts])


def rib_profile(s: np.ndarray, top: np.ndarray) -> np.ndarray:
    """Closed outline of a rib: bed from s[0] to s[-1], top edge along `top`."""
    line = LineString(np.column_stack([s, top])).simplify(0.02)
    pts = np.asarray(line.coords)
    return np.vstack([[s[0], 0.0], pts, [s[-1], 0.0]])


def opened(top: np.ndarray, step: float, args) -> np.ndarray:
    """Cut down raised parts shorter than `min_feature` along the rib. A grey opening,
    which only ever lowers the top, so the jig never moves closer to the robot."""
    n = int(np.ceil(args.min_feature / step)) + 1
    return ndimage.grey_opening(top, size=n, mode="constant", cval=0.0)


def profile(
    grid, along_x: bool, pos: float, w: float, lo: float, hi: float, args, connector: float = 0.0, tie_height=None
):
    """Top edge of one rib, along X at Y = pos or along Y at X = pos, from lo to hi.

    Support: where the robot's underside is within reach above the rib, the rib rises to
    it (less the clearance). Locating fin: within `fin` along the rib of such a point the
    rib rises to the cradle depth, which holds the side of the link it is under. Further
    away the rib is only a tie, `tie_height` high (`frame` unless given), inside the
    band round the robot, and absent
    outside it. A wall along the split is a connector: within `connector` of the
    centreline it stays `dovetail_height` high, joining the two dovetail blocks. Under
    the bay a rib is only a `bay_tie` high tie."""
    s_all, across = (grid.xs, grid.ys) if along_x else (grid.ys, grid.xs)
    half = 0.5 * (across[1] - across[0])
    sel = (across >= pos - w / 2 - half) & (across <= pos + w / 2 + half)

    def take(a):
        return a[:, sel] if along_x else a[sel, :].T

    # The bay is applied below as a cap, so read the heights without it: a zeroed bay
    # here would spread one sample past its edge and cut the bay grid off.
    clear = ndimage.minimum_filter1d(take(grid.clear_raw).min(axis=1), 3)
    contact = take(grid.contact).any(axis=1)
    band = take(grid.band).any(axis=1)
    bay = take(grid.bay).any(axis=1)
    keep = (s_all >= lo) & (s_all <= hi)
    s, clear, contact, band, bay = s_all[keep], clear[keep], contact[keep], band[keep], bay[keep]
    step = float(s[1] - s[0])

    dist = ndimage.distance_transform_edt(~contact) * step if contact.any() else np.full(len(s), np.inf)
    tie = np.where(band, args.frame if tie_height is None else tie_height, 0.0)
    tie = np.where(np.abs(s) <= connector, args.dovetail_height, tie)
    top = np.minimum(np.where(dist <= args.fin, clear, tie), clear)
    # Under the bay only a low tie, well below the bridge PCB and its wiring.
    top[bay] = np.minimum(args.bay_tie, clear[bay])
    return s, opened(top, step, args)


def build_rib(s, top, w, place, args) -> list[m3.Manifold]:
    """A rib in its own plane (u along the rib, v up), solid, extruded w and placed.
    Where the top falls to the bed the rib is broken, and any piece shorter than
    `min_feature` is dropped."""
    step = float(np.median(np.diff(s)))
    solid = top > 0.3
    pieces = []
    i = 0
    while i < len(s):
        if not solid[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(s) and solid[j + 1]:
            j += 1
        if s[j] - s[i] >= args.min_feature - 2 * step:
            ss, tt = s[i : j + 1], top[i : j + 1]
            sec = cross_section(rib_profile(ss, tt))
            pieces.append(m3.Manifold.extrude(sec, w).transform(place))
        i = j + 1
    return pieces


def place_x(y0: float, w: float) -> np.ndarray:
    # (u, v, e) -> X = u, Y = y0 + w/2 - e, Z = v
    return np.array([[1, 0, 0, 0], [0, 0, -1, y0 + w / 2], [0, 1, 0, 0]], float)


def place_y(x0: float, w: float) -> np.ndarray:
    # (u, v, e) -> X = x0 - w/2 + e, Y = u, Z = v
    return np.array([[0, 0, 1, x0 - w / 2], [1, 0, 0, 0], [0, 1, 0, 0]], float)


def prism(poly_xy: np.ndarray, height: float) -> m3.Manifold:
    return m3.Manifold.extrude(cross_section(poly_xy), height)


def box(x0, x1, y0, y1, height) -> m3.Manifold:
    return prism(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]), height)


def region_top(xs, ys, allowed, x0, x1, y0, y1) -> float:
    rows = (xs >= x0 - 0.5) & (xs <= x1 + 0.5)
    cols = (ys >= y0 - 0.5) & (ys <= y1 + 0.5)
    return float(allowed[np.ix_(rows, cols)].min())


def union(parts: list[m3.Manifold]) -> m3.Manifold:
    if hasattr(m3.Manifold, "batch_boolean"):
        return m3.Manifold.batch_boolean(parts, m3.OpType.Add)
    return reduce(lambda a, b: a + b, parts)


def to_trimesh(solid: m3.Manifold) -> trimesh.Trimesh:
    mesh = solid.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


# ---------------------------------------------------------------------------------------


def fill_rows(mask: np.ndarray) -> np.ndarray:
    """Fill each row (constant X) of a mask from its first set cell to its last."""
    out = mask.copy()
    for i in np.nonzero(mask.any(axis=1))[0]:
        cols = np.nonzero(mask[i])[0]
        out[i, cols[0] : cols[-1] + 1] = True
    return out


def choose_split(xs, h_without, ztop, lo, hi):
    """A point in the widest run of rows with nothing inside the cradle depth, looked for
    in the middle of the robot's length: the gap under the neck. Taken nearer the trunk
    end of the gap, since the body section is the longer one."""
    clear = (h_without >= ztop).all(axis=1)
    best = (0, None)
    i = 0
    n = len(xs)
    while i < n:
        if clear[i] and lo <= xs[i] <= hi:
            j = i
            while j + 1 < n and clear[j + 1] and xs[j + 1] <= hi:
                j += 1
            if j - i > best[0]:
                best = (j - i, xs[i] + 0.3 * (xs[j] - xs[i]))
            i = j + 1
        else:
            i += 1
    if best[1] is None:
        raise SystemExit("no clear row to split the jig at")
    return round(best[1], 1)


def main() -> None:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    model, data = load_robot(args.mjcf)
    geoms = visual_geoms(model)
    battery = {g for g in geoms if mesh_name(model, g) == "battery"}

    meshes = {g: geom_mesh_jig(model, data, g) for g in geoms}
    body_pts = np.vstack([v for g, (v, _) in meshes.items() if g not in battery])
    z_shift = args.base_gap - body_pts[:, 2].min()  # lowest point of the robot at base_gap
    lo, hi = body_pts.min(axis=0), body_pts.max(axis=0)
    x_shift = -(lo[0] - args.margin)  # jig X from 0 at the foot end
    x_lo, x_hi = 0.0, hi[0] - lo[0] + 2 * args.margin
    y_half = max(-lo[1], hi[1]) + args.margin

    res = args.res
    xs = np.arange(x_lo - 1.0, x_hi + 1.0 + res / 2, res)
    ys = np.arange(-y_half - 1.0, y_half + 1.0 + res / 2, res)

    # Ray-cast in the posed MJCF frame, so shift grid coordinates back.
    print(f"ray casting {len(xs)} x {len(ys)} grid ...", flush=True)
    h_with, h_without, hit = height_map(model, data, xs - x_shift, ys, body_pts[:, 2].min(), battery)
    h_with = np.where(h_with < BIG / 2, h_with + z_shift, BIG)
    h_without = np.where(h_without < BIG / 2, h_without + z_shift, BIG)

    ztop = args.base_gap + args.top
    length = x_hi - x_lo
    split = choose_split(xs, h_without, ztop, x_lo + 0.4 * length, x_lo + 0.75 * length)
    ztop_rows = np.where(xs >= split, args.base_gap + args.top_head, ztop)
    clear, clear_raw, band, footprint, hit_names, bay = allowed_height(
        args, h_with, h_without, hit, model, battery, ztop_rows
    )
    # Span the ties across the robot on every row, so the gap between the legs is tied
    # across and each leg's lattice is braced against the other, not only via the trunk.
    band = fill_rows(band)
    # Where a rib under the robot can reach its underside: support and the root of a fin.
    contact = footprint & ~bay & (clear < ztop_rows[:, None] - 0.01)
    grid = SimpleNamespace(xs=xs, ys=ys, clear=clear, clear_raw=clear_raw, contact=contact, band=band, bay=bay)
    print(f"split at X = {split} mm, jig {length:.1f} x {2 * y_half:.1f} mm")

    w, sp = args.rib, args.spine
    y_lo, y_hi = -y_half, y_half

    # Longitudinal ribs, symmetric about the centreline. No outer frame.
    n = int(np.floor((y_half - w) / args.pitch))
    y_ribs = [k * args.pitch for k in range(-n, n + 1)]

    def section(xa: float, xb: float, end_wall_x: float, spine_x: float, tie=None) -> m3.Manifold:
        # Transverse ribs on a pitch measured from the split, and the wall along the split.
        step = args.pitch if end_wall_x < spine_x else -args.pitch
        x_ribs = []
        xr = spine_x - np.sign(step) * (sp / 2 + args.pitch - w / 2)
        while (xr - end_wall_x) * np.sign(step) > args.pitch / 2:
            x_ribs.append(xr)
            xr -= step
        widths = {x0: w for x0 in x_ribs}
        widths[spine_x] = sp
        prof_x = {y0: profile(grid, True, y0, w, xa, xb, args, tie_height=tie) for y0 in y_ribs}
        prof_y = {
            x0: profile(grid, False, x0, wd, y_lo, y_hi, args, connector=span if x0 == spine_x else 0.0, tie_height=tie)
            for x0, wd in widths.items()
        }
        parts = []
        for y0, (s_, t_) in prof_x.items():
            parts += build_rib(s_, t_, w, place_x(y0, w), args)
        for x0, (s_, t_) in prof_y.items():
            parts += build_rib(s_, t_, widths[x0], place_y(x0, widths[x0]), args)
        return union(parts)

    # Dovetails: tabs on the head section, sockets in the body section, one each side of
    # the centreline. Each goes as far out as it can while at least half of its block
    # still sits in the lattice (the band round the robot), so the joints are as far
    # apart as the structure allows without walls running out to reach them. A block must
    # not come within reach of the robot or the bay; under the neck there is room, as it
    # sits well above the blocks.
    dt_depth, dt_root, dt_tip = 6.0, 6.0, 10.0
    blk_w, blk_body, blk_head = dt_tip + 6.0, dt_depth + 4.0, 6.0
    rows = (xs >= split - blk_body) & (xs <= split + blk_head)

    def fits(c: float) -> bool:
        room = region_top(xs, ys, clear, split - blk_body - 1, split + blk_head + 1, c - blk_w / 2 - 1, c + blk_w / 2 + 1)
        cols = (ys >= c - blk_w / 2) & (ys <= c + blk_w / 2)
        return room >= args.dovetail_height + 0.5 and band[np.ix_(rows, cols)].mean() >= 0.5

    candidates = [
        float(yc) for yc in np.arange(blk_w / 2 + 4.0, y_half - blk_w / 2 + 0.01, 0.5) if fits(yc) and fits(-yc)
    ]
    if not candidates:
        raise SystemExit("no room for the dovetails at the split")
    dt_y = max(candidates)
    span = dt_y + blk_w / 2

    body = section(x_lo, split, x_lo + w / 2, split - sp / 2)
    # The head section's ties, from the joints up to the head, stand as tall as the joints.
    head = section(split, x_hi, x_hi - w / 2, split + sp / 2, tie=args.dovetail_height)

    report_dt = []
    for side in (-1, 1):
        yc = side * dt_y
        yb0, yb1 = yc - blk_w / 2, yc + blk_w / 2
        top = min(args.dovetail_height, region_top(xs, ys, clear, split - blk_body, split + blk_head, yb0, yb1))
        report_dt.append({"y_centre": yc, "height": top})
        tab = np.array(
            [
                [split + 0.5, yc - dt_root / 2],
                [split - dt_depth, yc - dt_tip / 2],
                [split - dt_depth, yc + dt_tip / 2],
                [split + 0.5, yc + dt_root / 2],
            ]
        )
        tab_poly = Polygon(tab)
        socket_poly = tab_poly.buffer(args.fit, join_style="mitre")
        socket = prism(np.asarray(orient(socket_poly, 1.0).exterior.coords)[:-1], top + 2.0).translate([0, 0, -1.0])
        head = head + box(split, split + blk_head, yb0, yb1, top) + prism(tab, top)
        body = body + box(split - blk_body, split, yb0, yb1, top)
        body = body - socket

    body_mesh, head_mesh = to_trimesh(body), to_trimesh(head)

    # Checks: one piece each, fits the bed, sections do not overlap, robot clear.
    report = {"parameters": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()}}
    report["split_x_mm"] = split
    report["dovetails"] = report_dt
    for name, mesh, solid in (("body", body_mesh, body), ("head", head_mesh, head)):
        ext = mesh.bounds[1] - mesh.bounds[0]
        pieces = len(solid.decompose())
        report[name] = {
            "size_mm": [round(float(e), 1) for e in ext],
            "volume_cm3": round(mesh.volume / 1000, 1),
            "pla_g": round(mesh.volume / 1000 * 1.24, 0),
            "pieces": pieces,
            "manifold": str(solid.status()),
            "genus": int(solid.genus()),
            "fits_bed": bool(ext[0] <= args.bed and ext[1] <= args.bed),
        }
        mesh.export(args.out / f"jig_{name}.stl")
    # Smallest vertical gap from any jig vertex up to the robot (battery left out).
    gaps = []
    for mesh in (body_mesh, head_mesh):
        v = mesh.vertices
        i = np.clip(np.round((v[:, 0] - xs[0]) / res).astype(int), 0, len(xs) - 1)
        j = np.clip(np.round((v[:, 1] - ys[0]) / res).astype(int), 0, len(ys) - 1)
        under = h_without[i, j] < BIG / 2
        gaps.append(float((h_without[i, j] - v[:, 2])[under].min()))
    report["min_vertical_gap_mm"] = round(min(gaps), 3)
    report["overlap_cm3"] = round(float(to_trimesh(body ^ head).volume) / 1000, 4)

    # Held area: where the cradle surface follows the robot within 0.75 mm of the
    # clearance, by the link it meets. Ribs stand on a fraction of it.
    near = contact & (clear >= h_without - args.clearance - 0.75)
    bodies = np.array([model.body(model.geom_bodyid[g]).name if g >= 0 else "" for g in range(-1, model.ngeom)])
    hit_body = bodies[hit + 1]
    contact = {}
    for b in np.unique(hit_body[footprint]):
        sel = (hit_body == b) & near
        contact[b] = round(float(sel.sum() * res * res), 1)
    report["contact_area_mm2_by_body"] = dict(sorted(contact.items(), key=lambda kv: -kv[1]))
    report["robot_lowest_z_mm"] = args.base_gap
    report["bay_keep_out_mm2"] = round(float(bay.sum() * res * res), 0)
    ii, jj = np.nonzero(bay)
    report["bay_keep_out_extent_mm"] = {
        "x": [round(float(xs[ii].min()), 1), round(float(xs[ii].max()), 1)],
        "y": [round(float(ys[jj].min()), 1), round(float(ys[jj].max()), 1)],
    }
    report["robot_transform"] = {"rotation": R_JIG.tolist(), "translate_mm": [x_shift, 0.0, z_shift]}
    (args.out / "jig_report.json").write_text(json.dumps(report, indent=2) + "\n")
    keys = ("split_x_mm", "body", "head", "overlap_cm3", "min_vertical_gap_mm", "dovetails", "bay_keep_out_extent_mm")
    print(json.dumps({k: report[k] for k in keys}, indent=2))
    print("contact mm2 by link:", report["contact_area_mm2_by_body"])

    if args.preview:
        write_preview(args, model, meshes, battery, x_shift, z_shift, body_mesh, head_mesh)


def write_preview(args, model, meshes, battery, x_shift, z_shift, body_mesh, head_mesh) -> None:
    """GLB with the robot (by body) and both jig sections, all in jig mm."""
    scene = trimesh.Scene()
    by_body: dict[str, list[trimesh.Trimesh]] = {}
    for g, (v, f) in meshes.items():
        if g in battery:
            continue
        rgba = model.mat_rgba[model.geom_matid[g]] if model.geom_matid[g] >= 0 else model.geom_rgba[g]
        mesh = trimesh.Trimesh(v + [x_shift, 0.0, z_shift], f, process=True)
        if len(mesh.faces) > 3000:
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=int(len(mesh.faces) * 0.4))
            except Exception as exc:  # fast-simplification missing: keep full detail
                print("no decimation:", exc)
        mesh.visual.face_colors = (np.clip(rgba, 0, 1) * 255).astype(np.uint8)
        by_body.setdefault(model.body(model.geom_bodyid[g]).name, []).append(mesh)
    for name, parts in by_body.items():
        mesh = trimesh.util.concatenate(parts)
        scene.add_geometry(mesh, geom_name=f"robot:{name}", node_name=f"robot:{name}")
    scene.add_geometry(body_mesh, geom_name="jig_body", node_name="jig_body")
    scene.add_geometry(head_mesh, geom_name="jig_head", node_name="jig_head")
    scene.export(args.out / "jig_preview.glb")
    # The same GLB as base64 text, which the published 3D view fetches and decodes:
    # artifact pages serve neither .glb nor a fetch of a data: URL.
    import base64

    (args.out / "jig_preview.txt").write_bytes(base64.b64encode((args.out / "jig_preview.glb").read_bytes()))


if __name__ == "__main__":
    main()
