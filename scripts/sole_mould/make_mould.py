"""Mould-making parts for casting the foot soles in polyurethane.

The silicone mould is cast around a printed master of the sole (the sole STL
printed on a resin printer). This script makes the printed parts that go around
the master, not the mould cavities themselves:

    cup_frame      Frame for the bottom (cup) silicone half, which forms the sole's
                   outside. The silicone stays in it as a rigid backing, held by a
                   lip around the back edge. Two sockets take the plug frame's pegs.
    plug_frame     Frame for the top (plug) silicone half, which forms the pocket
                   and the rim top. Two pegs of different diameters, so it only
                   goes on one way round, long enough to engage before the plug
                   enters the cup. Cross bars at its top are cast into the
                   silicone, which fills holes through them, so the middle of the
                   plug hangs from the bars instead of sagging between the lips.
    parting_plate  Base for the first pour. The master sits rim-down on it, held by
                   a boss that fits its opening. Its face moulds the parting face
                   of the cup half: four key dimples, the land and the overflow
                   moat. Five push holes through the boss, one in each corner
                   and one in the middle, reach the inside of the master's
                   floor, so a rod pushed through them pops the master off the
                   boss and leaves it in the silicone.
    push_tool      Five pins on a base, one for each push hole, to push the master
                   off the boss in one go. Each pin is cut so all five reach the
                   master's sloping floor together, and the base stops on the
                   plate once the master is clear of the boss. Embossed L or R
                   on the pin side: the floors slope opposite ways, so the tools
                   are not interchangeable. Print it base down, pins up.
    moat_insert    Ring that fills the moat during the second pour, so the plug
                   silicone does not fill it; a press fit in the moat. Six tabs on
                   its top lie across the land and the full width of the master's
                   rim, and mould the vents into the plug's face. Six pins mould
                   risers through the plug from the moat to its top. Print it
                   upright, with supports from the bed under the tabs only: they
                   overhang about 4.7 mm.

The vents run across the whole wall top and stop at its inner edge, which the
plug's core moulds. The core and the vents are both in the plug, so the vents
always open into the cavity, however far the plug sits off centre.

The vents are in the plug's face, so their floor is level with the top of the
sole wall: resin only reaches them once the cavity is full, and air, which
gathers at the top, goes first. Overflow across the land into the moat is at the
same level. The plug's face covers the moat, so the risers let the moat's air
out, hold more overflow, and show when the moat is filling.

The parting plane is the rim top, which on these soles is tilted to the sole's
bottom. Everything is built in a mould frame with the rim plane at z = 0, the
part below it and the pocket opening upwards. Cast with the mould standing on a
level surface and the rim plane is level.

Steps:
    1. Press the master rim-down onto the parting plate's boss. If it is loose
       on the boss, add a little plasticine or Blu Tack between the plate and
       the rim, not glue, so the plate comes away from the master in step 2.
       Put tape over the push holes underneath, so silicone that creeps under
       the rim cannot run out of them. Set the cup frame, lip edge up, into the plate's fence and seal
       its outside edge. Pour the cup silicone to the top of the frame and
       leave it to cure fully, undisturbed and level, for the silicone's full
       cure time (not just its demould time).
    2. Once cured, turn plate, frame and silicone over together and peel the
       plate off from one edge. Take the tape off and press the push tool
       (or a 4 mm rod through each hole in turn, corners first) into the push
       holes until its base meets the plate, to pop the master off the boss so it stays in the silicone, pocket up. If it comes
       away with the plate anyway, pull it off the plate and press it
       back into its cavity, one end first so the air gets out. Check with a
       straight edge that the rim is flush with the silicone face, since
       anything under the master leaves it proud and puts a step in the
       parting line. Press the moat insert into the moat, coat the
       silicone face with release (silicone sticks to silicone), put the plug
       frame on its pegs and pour the plug silicone through the windows
       between the bars, filling the master's pocket first, then up through
       the bar holes until it is level with the tops of the bars. Leave it to
       cure fully as well.
    3. Separate the halves, take out the master and the insert. To cast: pour the
       dose into the cup, lower the plug frame on its pegs and weight it. The
       frames meet face to face, which sets the wall thickness. Resin
       appearing in the risers means the moat is filling.

    uv run --with manifold3d --with trimesh --with shapely --with numpy --with matplotlib \\
        scripts/sole_mould/make_mould.py hardware/cad/stl/sole_left.stl --out hardware/cad/sole_mould/left

Writes the five parts as STLs oriented for printing, the two silicone halves and
the cast (for checking), a section drawing, and a summary of volumes.
"""

import argparse
import json
import math
from pathlib import Path

import manifold3d as m3
import numpy as np
import trimesh
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import nearest_points, unary_union

P = dict(
    land=2.5,          # contact land between cavity edge and moat, at the parting face [mm]
    land_draft=2.0,    # the moat's inner wall slopes out this much over its depth, so
                       # the silicone under the land is land + land_draft wide at its
                       # root: stronger, and open enough for the first pour to fill
    moat_w=6.0,        # overflow moat width at the parting face
    moat_d=4.0,        # moat depth
    vent_w=2.0,        # vent width
    vent_d=0.6,        # vent height above the top of the sole wall, in the plug's
                       # face (three 0.2 mm layers on the insert's tabs)
    riser_d=4.0,       # risers from the moat up through the plug
    bar_w=6.0,         # plug frame cross bars: width
    bar_h=4.0,         # depth below the top of the plug frame
    bar_ch=1.5,        # 45 degree chamfers under the bar, so bubbles slide off
    bar_hole_d=4.0,    # holes through the bars, filled by the silicone
    bar_hole_pitch=10.0,
    bars_x=(-0.5, 0.5),  # bars across the short side, at these fractions of the half-length
    bars_y=(0.0,),       # bars along the long side, at these fractions of the half-width
    boss_h=1.5,        # master locating boss height
    push_d=5.0,        # push holes through the plate's boss, to pop the master off
    push_inset=7.0,    # corner push holes' centres in from the edge of the opening
    push_clear=0.5,    # push tool pins in the push holes (loose)
    push_stroke=1.6,   # how far the push tool moves the master: just off the 1.5 mm boss
    push_base_t=5.0,   # push tool base thickness
    # Clearances are per side (gap between the two surfaces), sized for FDM:
    # 0.25 for a sliding fit, 0.15 for a press fit.
    boss_clear=0.15,   # boss inset from the master's opening (press fit)
    insert_clear=0.15, # moat insert in its silicone groove (press fit)
    key_r=3.0,         # key hemisphere radius
    margin=4.0,        # frame inner wall beyond the moat's outer edge (4.0 keeps the
                       # frames printed with the 1.5 mm land and 5 mm moat, at margin 6)
    floor_sil=8.0,     # cup silicone under the deepest point of the part
    top_sil=10.0,      # plug silicone above the parting plane
    frame_t=3.0,       # frame wall thickness
    lip_w=3.0,         # lip overhang holding the silicone in its frame
    lip_h=2.0,         # lip thickness (plus a 45 degree chamfer of lip_w)
    lug=18.0,          # peg lug size
    pegs=((1, 1, 10.0), (-1, -1, 9.0)),  # (x side, y side, diameter)
    peg_clear=0.25,    # peg in socket (sliding fit)
    plate_t=3.0,       # parting plate deck thickness
    fence_h=3.0,       # fence locating the cup frame on the plate
    fence_t=2.5,
    fence_clear=0.3,   # cup frame in the plate's fence (loose: only centres it)
    overfill=0.20,     # cast dose above the part volume
    pu_density=1.05,   # g/cm^3, typical urethane rubber
    si_density=1.10,   # g/cm^3, typical platinum silicone
)
SEG = 64


# ---------------------------------------------------------------- conversions

def to_manifold(tm):
    mesh = m3.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                   tri_verts=np.asarray(tm.faces, np.uint32))
    m = m3.Manifold(mesh)
    if m.status() != m3.Error.NoError:
        raise SystemExit(f"input mesh is not a manifold: {m.status()}")
    return m


def to_trimesh(m, name="an output mesh"):
    mesh = m.to_mesh()
    # Manifold's output is closed as indexed. Leave it unprocessed: merging by
    # position folds the zero-area triangles the booleans leave along a few
    # edges, which would show as small loose bodies.
    tm = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts),
                         process=False)
    if not tm.is_watertight:
        raise SystemExit(f"{name} is not watertight")
    return tm


def cs_from(geom):
    geoms = geom.geoms if isinstance(geom, MultiPolygon) else [geom]
    loops = []
    for g in geoms:
        if g.is_empty:
            continue
        loops.append(np.asarray(g.exterior.coords)[:-1])
        loops += [np.asarray(r.coords)[:-1] for r in g.interiors]
    return m3.CrossSection(loops, m3.FillRule.EvenOdd)


def shapely_from(cs):
    geom = Polygon()
    for loop in cs.to_polygons():
        if len(loop) >= 3:
            geom = geom.symmetric_difference(Polygon(loop).buffer(0))
    return geom


def extrude(geom, z0, z1):
    return cs_from(geom).extrude(z1 - z0).translate((0, 0, z0))


def largest(geom):
    geoms = geom.geoms if isinstance(geom, MultiPolygon) else [geom]
    return max(geoms, key=lambda g: g.area)


# ---------------------------------------------------------------- rim plane

def rim_frame(tm):
    """Transform taking the sole into the mould frame: rim plane at z = 0,
    pocket opening towards +z, outline centred on the origin."""
    n_f, a_f = tm.face_normals, tm.area_faces
    best = None
    for s in (1.0, -1.0):
        nz = s * n_f[:, 2]
        tilt = np.degrees(np.arccos(np.clip(nz, -1, 1)))
        band = (tilt > 0.5) & (tilt < 15)
        if not band.any():
            continue
        keys = np.round(s * n_f[band] / 0.004).astype(int)
        uniq, inv = np.unique(keys, axis=0, return_inverse=True)
        w = np.bincount(inv.ravel(), weights=a_f[band])
        k = np.argmax(w)
        n = np.average(s * n_f[band][inv.ravel() == k], axis=0, weights=a_f[band][inv.ravel() == k])
        n = s * n / np.linalg.norm(n)  # back to the outward normal of the rim faces
        if best is None or w[k] > best[0]:
            best = (w[k], n)
    area, n = best
    # the rim top is the part's extreme in direction n
    h = tm.vertices @ n
    top = h.max()
    on_rim = np.abs(h - top) < 0.02
    # rotation taking n to +z
    z = np.array([0, 0, 1.0])
    v, c = np.cross(n, z), float(n @ z)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx / (1 + c)
    T = np.eye(4)
    T[:3, :3] = R
    T[2, 3] = -top
    tilt = math.degrees(math.acos(abs(c)))
    return T, area, int(on_rim.sum()), tilt


# ---------------------------------------------------------------- parts

def frame_ring(inner, outer_fp):
    return outer_fp.difference(inner)


def lip(inner_w, inner_l, z_face, up):
    """Lip around the inside of a frame at z_face, with a 45 degree chamfer on
    the side facing the silicone. up=True: chamfer above the lip."""
    lw, lh = P["lip_w"], P["lip_h"]
    # 0.05 mm into the frame wall so the union fuses instead of touching
    full = m3.CrossSection.square((inner_w + 0.1, inner_l + 0.1), center=True)
    inset_dims = (inner_w - 2 * lw, inner_l - 2 * lw)
    inset = m3.CrossSection.square(inset_dims, center=True)
    h = lh + lw + 0.1  # past where the chamfer meets the wall, so the cut trims the top
    solid = full.extrude(h)
    # 45 degree chamfer, run 0.2 mm past the lip top and the wall so no edge of
    # the cut lies exactly on a face of the frame
    e = 0.2
    hole = inset.extrude(lh + 0.01) + inset.extrude(
        lw + e, scale_top=((inset_dims[0] + 2 * (lw + e)) / inset_dims[0],
                           (inset_dims[1] + 2 * (lw + e)) / inset_dims[1])).translate((0, 0, lh))
    part = solid - hole
    if up:
        return part.translate((0, 0, z_face))
    return part.mirror((0, 0, 1)).translate((0, 0, z_face))


def label_shape(text, height):
    """Outline of text as a CrossSection, centred on the origin."""
    from matplotlib.textpath import TextPath
    from matplotlib.font_manager import FontProperties
    tp = TextPath((0, 0), text, size=height, prop=FontProperties(family="DejaVu Sans", weight="bold"))
    cs = m3.CrossSection([np.asarray(poly) for poly in tp.to_polygons()], m3.FillRule.EvenOdd)
    b = cs.bounds()
    return cs.translate(((-(b[0] + b[2]) / 2), -(b[1] + b[3]) / 2))


def build(stl, out, label=None):
    tm = trimesh.load(stl, force="mesh")
    T, rim_area, rim_verts, tilt = rim_frame(tm)
    tm.apply_transform(T)
    part = to_manifold(tm)
    zmin = part.bounding_box()[2]

    # outlines at the parting plane
    outer = unary_union([Polygon(largest(shapely_from(part.slice(z))).exterior)
                         for z in (-0.05, -0.5, -1.0)])
    opening = max(largest(shapely_from(part.slice(-P["boss_h"]))).interiors,
                  key=lambda r: Polygon(r).area)
    opening = Polygon(opening)
    # inner edge of the wall top
    rim_in = Polygon(max(largest(shapely_from(part.slice(-0.002))).interiors,
                         key=lambda r: Polygon(r).area))
    bx = (outer.bounds[0] + outer.bounds[2]) / 2
    by = (outer.bounds[1] + outer.bounds[3]) / 2
    part = part.translate((-bx, -by, 0))
    tm.apply_translation((-bx, -by, 0))
    from shapely.affinity import translate as stranslate
    outer = stranslate(outer, -bx, -by)
    opening = stranslate(opening, -bx, -by)
    rim_in = stranslate(rim_in, -bx, -by)

    moat_out = outer.buffer(P["land"] + P["moat_w"], join_style="round")
    md, dr = P["moat_d"], P["land_draft"]

    def land_solid(extra):
        # the land and the silicone under it: outer + land at the parting face,
        # outer + land + land_draft at the moat floor, run 0.5 mm past both so
        # the cuts leave no coincident faces. The outline is convex, so a hull
        # of the two ends is the sloped wall.
        def at(z):
            off = P["land"] + extra + dr * (-z / md)
            return extrude(outer.buffer(off, join_style="round"), z - 0.001, z)
        return (at(0.5) + at(-md - 0.5)).hull()

    moat_vol = extrude(moat_out, -md, 0) - land_solid(0)
    insert_ring = extrude(outer.buffer(P["land"] + P["moat_w"] - P["insert_clear"]), -md, 0) \
        - land_solid(P["insert_clear"])

    # frame inner rectangle and keys in its corners
    x0, y0, x1, y1 = moat_out.bounds
    margin = P["margin"]
    while True:
        Xi, Yi = (x1 - x0) / 2 + margin, (y1 - y0) / 2 + margin
        kr = P["key_r"]
        keys = [(sx * (Xi - kr - 1.5), sy * (Yi - kr - 1.5)) for sx in (1, -1) for sy in (1, -1)]
        if min(Point(k).distance(moat_out) for k in keys) >= kr + 1.0:
            break
        margin += 0.5
    inner = box(-Xi, -Yi, Xi, Yi)
    ft, lug = P["frame_t"], P["lug"]
    outer_fp = box(-Xi - ft, -Yi - ft, Xi + ft, Yi + ft)
    lugs = []
    for sx, sy, d in P["pegs"]:
        # 1 mm off the inner corner: a lug touching it at one point leaves a
        # degenerate polygon
        cxl, cyl = sx * (Xi + 1 + lug / 2), sy * (Yi + 1 + lug / 2)
        lugs.append((cxl, cyl, d))
        outer_fp = outer_fp.union(box(cxl - lug / 2, cyl - lug / 2, cxl + lug / 2, cyl + lug / 2))

    # vents: four corners and the middle of each long side, from the inner edge
    # of the wall top to the middle of the moat, where a riser goes up. The
    # wall's inner edge is the plug core's, so a vent stays open to the cavity
    # whatever the misalignment. Cut 0.02 mm past it rather than leave a
    # feather of silicone at the core's edge.
    vents, risers = [], []
    ox0, oy0, ox1, oy1 = outer.bounds
    targets = [(ox1 + 9, oy1 + 9), (ox0 - 9, oy1 + 9), (ox1 + 9, oy0 - 9), (ox0 - 9, oy0 - 9)]
    long_y = (oy1 - oy0) >= (ox1 - ox0)
    targets += [(ox1 + 9, 0), (ox0 - 9, 0)] if long_y else [(0, oy1 + 9), (0, oy0 - 9)]
    for t in targets:
        p = nearest_points(outer.exterior, Point(t))[0]
        d = np.array([t[0] - p.x, t[1] - p.y])
        d /= np.linalg.norm(d)
        q = np.array([p.x, p.y]) + d * (P["land"] + P["moat_w"] / 2)
        seg = LineString([(p.x - d[0] * 6, p.y - d[1] * 6), tuple(q)])
        vents.append(seg.buffer(P["vent_w"] / 2, cap_style="flat").difference(rim_in.buffer(-0.02)))
        risers.append(tuple(q))
    vents = unary_union(vents)
    if vents.geom_type != "MultiPolygon" or len(vents.geoms) != len(targets):
        raise SystemExit("a vent did not clip to one piece")

    # heights
    H_A = math.ceil(-zmin + P["floor_sil"])
    H_B = P["top_sil"]
    peg_len = math.ceil(-zmin) + 2
    sock_depth = peg_len + 1.5
    if sock_depth > H_A - P["lip_h"]:
        raise SystemExit("sockets deeper than the cup frame")

    # cup frame, z in [-H_A, 0]
    cup_frame = extrude(frame_ring(inner, outer_fp), -H_A, 0) + lip(2 * Xi, 2 * Yi, -H_A, up=True)
    for cxl, cyl, d in lugs:
        cup_frame -= m3.Manifold.cylinder(sock_depth + 0.1, d / 2 + P["peg_clear"], circular_segments=SEG) \
            .translate((cxl, cyl, -sock_depth))

    # plug frame, z in [0, H_B], pegs below
    plug_frame = extrude(frame_ring(inner, outer_fp), 0, H_B) + lip(2 * Xi, 2 * Yi, H_B, up=False)
    for cxl, cyl, d in lugs:
        ch = 1.0
        peg = m3.Manifold.cylinder(peg_len - ch, d / 2, circular_segments=SEG).translate((0, 0, ch)) + \
            m3.Manifold.cylinder(ch, d / 2 - ch, d / 2, circular_segments=SEG)
        peg += m3.Manifold.cylinder(1.0, d / 2, circular_segments=SEG).translate((0, 0, peg_len - 0.5))
        plug_frame += peg.translate((cxl, cyl, -peg_len))

    # cross bars at the top of the plug frame, cast into the silicone
    bw, bh, bc = P["bar_w"], P["bar_h"], P["bar_ch"]
    # built 1 mm proud and trimmed with the frame below, so the bar tops are not
    # a second face lying in the frame's top face
    profile = m3.CrossSection([np.array([(-bw / 2, 1), (bw / 2, 1), (bw / 2, -bh + bc), (bw / 2 - bc, -bh),
                                         (-bw / 2 + bc, -bh), (-bw / 2, -bh + bc)])], m3.FillRule.EvenOdd)
    bars = m3.Manifold()
    holes = []
    for f in P["bars_x"]:   # along x, at y = f * Yi
        L = 2 * Xi + 2  # 1 mm into each wall
        bar = profile.extrude(L).transform(np.array([[0, 0, 1, -L / 2], [1, 0, 0, f * Yi], [0, 1, 0, H_B]], float))
        bars += bar
        holes += [(x, f * Yi) for x in np.arange(-Xi + 6, Xi - 5.9, P["bar_hole_pitch"])]
    for f in P["bars_y"]:   # along y, at x = f * Xi
        L = 2 * Yi + 2
        bar = profile.extrude(L).transform(np.array([[1, 0, 0, f * Xi], [0, 0, -1, L / 2], [0, 1, 0, H_B]], float))
        bars += bar
        holes += [(f * Xi, y) for y in np.arange(-Yi + 6, Yi - 5.9, P["bar_hole_pitch"])]
    crossings = [(f2 * Xi, f1 * Yi) for f1 in P["bars_x"] for f2 in P["bars_y"]]
    for hx, hy in holes:
        if min(math.hypot(hx - cx, hy - cy) for cx, cy in crossings) < bw:
            continue  # leave the crossings solid
        bars -= m3.Manifold.cylinder(bh + 2, P["bar_hole_d"] / 2, circular_segments=SEG) \
            .translate((hx, hy, H_B - bh - 1))
    for rx, ry in risers:  # clear the riser pins
        bars -= m3.Manifold.cylinder(bh + 2, P["riser_d"] / 2 + 1.5, circular_segments=SEG) \
            .translate((rx, ry, H_B - bh - 1))
    plug_frame = (plug_frame + bars) ^ extrude(outer_fp.buffer(1), -peg_len - 1, H_B)

    # parting plate (M frame: deck above z = 0, features below)
    fence_in = outer_fp.buffer(P["fence_clear"], join_style="mitre")
    fence_out = fence_in.buffer(P["fence_t"], join_style="mitre")
    plate = extrude(fence_out, 0, P["plate_t"]) + extrude(fence_out.difference(fence_in), -P["fence_h"], 0)
    plate_feat = moat_vol
    for kx, ky in keys:
        plate_feat += m3.Manifold.sphere(P["key_r"], circular_segments=SEG).translate((kx, ky, 0))
    plate += plate_feat + extrude(opening.buffer(-P["boss_clear"]), -P["boss_h"], 0.01)
    # push holes: one in each corner of the opening and one in the middle
    ox0, oy0, ox1, oy1 = opening.bounds
    corner_ring = opening.buffer(-P["push_inset"]).exterior
    push = [nearest_points(corner_ring, Point(c))[0] for c in ((ox1, oy1), (ox0, oy1), (ox1, oy0), (ox0, oy0))]
    push.append(opening.centroid)
    for pt in push:
        if not opening.buffer(-P["boss_clear"] - 1.0).contains(pt.buffer(P["push_d"] / 2)):
            raise SystemExit("a push hole breaks out of the boss")
        plate -= m3.Manifold.cylinder(P["plate_t"] + P["boss_h"] + 2, P["push_d"] / 2, circular_segments=SEG) \
            .translate((pt.x, pt.y, -P["boss_h"] - 1))

    # push tool, in place with its base on the plate's back face (z = plate_t).
    # Each pin runs to where it first meets the master's floor, plus the stroke,
    # so all five touch together and the base stops the push.
    pin_r = P["push_d"] / 2 - P["push_clear"]
    zb, stroke = P["plate_t"], P["push_stroke"]
    pin_len = []
    tool = extrude(MultiPolygon([pt.buffer(1) for pt in push]).convex_hull.buffer(7, join_style="round"),
                   zb, zb + P["push_base_t"])
    for pt in push:
        col = part ^ m3.Manifold.cylinder(30, pin_r, circular_segments=SEG).translate((pt.x, pt.y, -29.9))
        if col.is_empty():
            raise SystemExit("a push hole does not reach the master's floor")
        z_hit = col.bounding_box()[5]
        length = zb - z_hit + stroke
        pin_len.append(round(length, 2))
        ch = 0.5
        pin = m3.Manifold.cylinder(length - ch + 0.5, pin_r, circular_segments=SEG).translate((0, 0, zb - length + ch)) + \
            m3.Manifold.cylinder(ch, pin_r - ch, pin_r, circular_segments=SEG).translate((0, 0, zb - length))
        tool += pin.translate((pt.x, pt.y, 0))
    if label:
        # on the pin side between the middle pin and one end, written mirrored so it
        # reads correctly once the tool is turned pins up
        ly = (push[0].y + push[4].y) / 2
        glyph = label_shape(label, 7.0).mirror((0, 1)).translate((push[4].x, ly))
        tool -= glyph.extrude(0.6 + 0.01).translate((0, 0, zb - 0.01))
    # checks: no clash with the plate when the base is home, and the pins only
    # just touch the master (to within the stroke) one stroke before that
    tool_clash = (tool ^ plate).volume()
    tool_early = (tool.translate((0, 0, stroke - 0.02)) ^ part).volume()
    if tool_clash > 1e-3 or tool_early > 1e-3:
        raise SystemExit(f"push tool fit: plate clash {tool_clash:.4f}, early contact {tool_early:.4f} mm3")

    vent_vol = extrude(vents, 0, P["vent_d"])
    riser_vol = m3.Manifold()
    for rx, ry in risers:
        riser_vol += m3.Manifold.cylinder(H_B + 1.0, P["riser_d"] / 2, circular_segments=SEG) \
            .translate((rx, ry, -0.01))
    insert = insert_ring + vent_vol + riser_vol

    # silicone halves
    # the rim lies exactly on the parting plane; lift the part 2 um through it
    # so the booleans split the pocket from the outside
    part_c = part.translate((0, 0, 0.002))
    lower = extrude(inner, -H_A, 0) - lip(2 * Xi, 2 * Yi, -H_A, up=True)
    regions = sorted((lower - part_c).decompose(), key=lambda m: -m.volume())
    cup_si = regions[0] - plate_feat
    pocket = m3.Manifold.batch_boolean(regions[1:], m3.OpType.Add) if len(regions) > 1 else m3.Manifold()
    key_bumps = m3.Manifold()
    for kx, ky in keys:
        key_bumps += m3.Manifold.sphere(P["key_r"], circular_segments=SEG).translate((kx, ky, 0))
    key_bumps = key_bumps ^ extrude(inner, -P["key_r"] - 1, 0)
    plug_si = (extrude(inner, 0, H_B) - lip(2 * Xi, 2 * Yi, H_B, up=False)) + pocket + key_bumps \
        - part_c - vent_vol - riser_vol - bars

    # checks: the halves must not overlap, and the space left between them
    # (less the moat, vents and risers) must be the part
    gap = (cup_si ^ plug_si).volume()
    cavity = (lower + extrude(inner, 0, H_B) - lip(2 * Xi, 2 * Yi, H_B, up=False)) \
        - cup_si - plug_si - moat_vol - vent_vol - riser_vol - bars
    out.mkdir(parents=True, exist_ok=True)

    def save(name, m, flip=False):
        if flip:
            m = m.rotate((180, 0, 0))
        zlo = m.bounding_box()[2]
        tmesh = to_trimesh(m.translate((0, 0, -zlo)), name)
        tmesh.export(out / f"{name}.stl")
        return tmesh

    save("print_cup_frame", cup_frame)
    save("print_plug_frame", plug_frame, flip=True)
    save("print_parting_plate", plate, flip=True)
    save("print_moat_insert", insert)
    save("print_push_tool", tool, flip=True)
    # check parts, in the mould frame
    to_trimesh(cup_si, "cup silicone").export(out / "check_cup_silicone.stl")
    to_trimesh(plug_si, "plug silicone").export(out / "check_plug_silicone.stl")
    to_trimesh(part, "cast").export(out / "check_cast.stl")

    dose = part.volume() / 1000 * (1 + P["overfill"])
    summary = {
        "source": str(stl),
        "rim_tilt_deg": round(tilt, 2),
        "rim_plane_area_mm2": round(rim_area, 1),
        "rim_plane_vertices": rim_verts,
        "part_depth_below_rim_mm": round(-zmin, 2),
        "push_tool_pin_lengths_mm": pin_len,
        "frame_inner_mm": [round(2 * Xi, 1), round(2 * Yi, 1)],
        "cup_frame_height_mm": H_A,
        "plug_frame_height_mm": H_B,
        "peg_length_mm": peg_len,
        "part_volume_cm3": round(part.volume() / 1000, 3),
        "cavity_volume_check_cm3": round(cavity.volume() / 1000, 3),
        "silicone_overlap_mm3": round(gap, 3),
        "cup_silicone_cm3": round(cup_si.volume() / 1000, 1),
        "plug_silicone_cm3": round(plug_si.volume() / 1000, 1),
        "silicone_total_g": round((cup_si.volume() + plug_si.volume()) / 1000 * P["si_density"], 0),
        "pu_dose_cm3": round(dose, 2),
        "pu_dose_g": round(dose * P["pu_density"], 1),
        "moat_capacity_cm3": round(moat_vol.volume() / 1000, 2),
        "riser_capacity_cm3": round((riser_vol ^ extrude(inner, 0, H_B)).volume() / 1000, 2),
        "params": P,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    section_drawing(out / "section.png", dict(cup_frame=cup_frame, plug_frame=plug_frame,
                                               cup_si=cup_si, plug_si=plug_si, cast=part,
                                               plate=plate, insert=insert), summary)
    return summary


def section_drawing(path, parts, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MPath

    colours = dict(cup_frame="#7a8aa0", plug_frame="#7a8aa0", cup_si="#e0a96d", plug_si="#c97b4a",
                   cast="#3a7d44", plate="#9aa7b8", insert="#b03a2e")

    def draw(ax, m, axis, colour, dz=0.0):
        if axis == "x":   # section in the plane y = 0, drawn as (x, z)
            cs = m.rotate((-90, 0, 0)).slice(0)
            loops = [np.asarray(l) for l in cs.to_polygons()]
        else:             # section in the plane x = 0, drawn as (y, z)
            cs = m.rotate((0, 90, 0)).slice(0)
            loops = [np.asarray(l)[:, ::-1] for l in cs.to_polygons()]
        verts, codes = [], []
        for l in loops:
            l = l + [0, dz]
            verts += list(l) + [l[0]]
            codes += [MPath.MOVETO] + [MPath.LINETO] * (len(l) - 1) + [MPath.CLOSEPOLY]
        if verts:
            ax.add_patch(PathPatch(MPath(verts, codes), facecolor=colour, edgecolor="k", lw=0.3))

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    for row, axis in enumerate(("x", "y")):
        ax = axes[row, 0]
        for k in ("cup_frame", "plug_frame", "cup_si", "plug_si", "cast"):
            draw(ax, parts[k], axis, colours[k])
        ax.set_title(f"casting: section at {'y' if axis == 'x' else 'x'} = 0")
        ax = axes[row, 1]
        # ready for the second pour: cup half with the master and the moat insert in it
        for k in ("cup_frame", "cup_si", "insert"):
            draw(ax, parts[k], axis, colours[k])
        draw(ax, parts["cast"], axis, "#dddddd")
        ax.set_title("before the second pour: cup half, master (grey), moat insert (red)")
        for a in axes[row]:
            a.set_aspect("equal")
            a.autoscale_view()
            a.grid(alpha=0.3)
    fig.suptitle(f"{Path(summary['source']).name}: rim tilt {summary['rim_tilt_deg']} deg, "
                 f"dose {summary['pu_dose_g']} g PU, silicone {summary['silicone_total_g']:.0f} g")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--label", help="letter on the push tool (default: L or R from the --out folder name)")
    args = ap.parse_args()
    label = args.label or {"left": "L", "right": "R"}.get(args.out.name.lower())
    s = build(args.stl, args.out, label)
    print(json.dumps({k: v for k, v in s.items() if k != "params"}, indent=2))


if __name__ == "__main__":
    main()
