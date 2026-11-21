"""Line drawing of Dukki for the README: black outlines over stepped grey shading, transparent background.

The robot is posed in the firmware's rest pose (microduck `dukki`, duck-control
`model::REST_POSITION`, recorded 21/11/2026) and left to settle on a floor in MuJoCo with
every joint held, so the trunk sits at the attitude the folded legs give it. The floor is
then dropped and only the visual meshes are drawn.

Geometry. The MJCF's visual meshes are cut down to 2,500 faces, too coarse to draw edges
from. Every printed part with a full-resolution export in `hardware/cad/stl/` (same part
frame, millimetres) is drawn from that export instead, matched by its bounding box; the
bought parts (J288s, bearings, electronics, battery) keep the MJCF's meshes.

How it is drawn, at `--supersample` times the output size and then scaled down:

    1. All posed meshes go into one static MuJoCo scene, which renders a geom id, a depth
       and the diffuse light per pixel.
    2. Fill: stepped shading, white and two greys (`--tones`), from one directional light
       (`--light`, upper left and in front by default). Each pixel takes the tone of the
       band its diffuse light falls in (`--steps`), so there are no gradients; with the
       black lines that is four colours.
    3. Outline: a thick line along the edge of the whole silhouette.
    4. Inner lines, thinner: where one part meets another (geom id changes), where a part
       folds behind itself (a jump in depth), and the creases of each mesh (adjacent faces
       more than `--crease` degrees apart), projected and kept where the depth buffer
       shows them.

    uv run --with mujoco --with trimesh --with scipy --with pillow \\
        python scripts/render/render_robot.py --out docs/pictures/dukki-render.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

ROBOT_DIR = Path("../microduck_rl/src/mjlab_microduck/robot/microduck")
CAD_STL = Path("hardware/cad/stl")

# duck-control model::REST_POSITION, in its joint order. The mouth is not in the MJCF.
REST_POSITION = {
    "left_hip_yaw": 0.1118,
    "left_hip_roll": 0.4862,
    "left_hip_pitch": -0.3475,
    "left_knee": 1.4747,
    "left_ankle": 0.5795,
    "neck_pitch": 0.5335,
    "head_pitch": 0.6061,
    "head_yaw": 0.0054,
    "head_roll": 0.0096,
    "right_hip_yaw": 0.0793,
    "right_hip_roll": -0.4218,
    "right_hip_pitch": 0.3234,
    "right_knee": -1.4908,
    "right_ankle": -0.4352,
}


def settle(scene: Path, seconds: float = 3.0) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The robot dropped onto the floor in the rest pose, every joint held there."""
    m = mujoco.MjModel.from_xml_path(str(scene))
    d = mujoco.MjData(m)
    d.qpos[2] = 0.25
    for name, q in REST_POSITION.items():
        d.qpos[m.jnt_qposadr[m.joint(name).id]] = q
        d.ctrl[m.actuator(name).id] = q
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    # Floor at z = 0 stays the reference; centre the robot over the origin.
    d.qpos[0:2] = 0.0
    mujoco.mj_kinematics(m, d)
    return m, d


def cad_meshes() -> list[tuple[np.ndarray, trimesh.Trimesh]]:
    """Every full-resolution CAD export, in metres, with its bounding box."""
    out = []
    for path in sorted(CAD_STL.glob("*.stl")):
        mesh = trimesh.load(path, force="mesh")
        mesh.apply_scale(0.001)
        out.append((mesh.bounds.copy(), mesh))
    return out


def mesh_file_vertices(m: mujoco.MjModel, mesh_id: int, spec: mujoco.MjSpec) -> trimesh.Trimesh:
    """The MJCF mesh as its asset file has it (the geom's frame before MuJoCo recentres it)."""
    path = ROBOT_DIR / spec.meshdir / spec.meshes[mesh_id].file
    mesh = trimesh.load(path, force="mesh")
    mesh.apply_scale(np.asarray(spec.meshes[mesh_id].scale))
    return mesh


def posed_meshes(m: mujoco.MjModel, d: mujoco.MjData, scene: Path, tol: float = 5e-4):
    """World-space mesh for every visual geom, full resolution where there is a CAD export."""
    spec = mujoco.MjSpec.from_file(str(scene))
    cad = cad_meshes()
    out, swapped = [], []
    for g in range(m.ngeom):
        if m.geom_group[g] != 2 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        raw = mesh_file_vertices(m, mid, spec)
        # Asset frame to the compiled geom frame: v_compiled = R(mesh_quat)^T (v - mesh_pos).
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, m.mesh_quat[mid])
        R = R.reshape(3, 3)
        p = m.mesh_pos[mid]
        mesh = raw
        # Mirrored pairs can share a bounding box, so check the surfaces too.
        best, best_err = None, tol
        for bounds, full in cad:
            if np.abs(bounds - raw.bounds).max() < tol:
                err = cKDTree(full.vertices).query(raw.vertices)[0].mean()
                if err < best_err:
                    best, best_err = full, err
        if best is not None:
            mesh = best
            swapped.append(m.mesh(mid).name)
        local = (mesh.vertices - p) @ R  # R^T (v - p), row vectors
        world = local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        out.append((g, trimesh.Trimesh(world, mesh.faces, process=True)))
    return out, sorted(set(swapped))


def look_at(eye: np.ndarray, target: np.ndarray, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Camera rotation (columns x right, y up, z back), MuJoCo's convention."""
    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)


def render_buffers(meshes, eye, target, fovy, size, reach, light_dir, crease_deg):
    """Geom index (-1 for none), depth along the view axis, and diffuse light, per pixel.

    `reach` is how far the robot extends from `target`: the depth range is kept to the robot
    alone, for the depth buffer's precision. The light is one directional light along
    `light_dir`, no ambient or specular, so the light is the cosine of the angle between the
    surface and the light. Normals are smoothed across faces less than `crease_deg` apart and
    kept sharp at the creases, as the lines are drawn.
    """
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    spec.visual.global_.offwidth = size[0]
    spec.visual.global_.offheight = size[1]
    spec.visual.quality.offsamples = 0
    spec.visual.headlight.active = 0
    spec.visual.headlight.ambient = [0, 0, 0]
    spec.visual.headlight.diffuse = [0, 0, 0]
    spec.visual.headlight.specular = [0, 0, 0]
    spec.add_material(name="matte", rgba=[1, 1, 1, 1], specular=0, shininess=0, reflectance=0)
    for i, (_, mesh) in enumerate(meshes):
        shaded = trimesh.graph.smooth_shade(mesh, angle=math.radians(crease_deg))
        mm = spec.add_mesh(name=f"m{i}")
        mm.uservert = shaded.vertices.astype(np.float32).ravel().tolist()
        mm.userface = shaded.faces.astype(np.int32).ravel().tolist()
        mm.usernormal = shaded.vertex_normals.astype(np.float32).ravel().tolist()
        mm.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
        spec.worldbody.add_geom(
            name=f"g{i}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m{i}",
            contype=0, conaffinity=0, material="matte")
    spec.worldbody.add_light(
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, dir=light_dir, pos=eye,
        diffuse=[1, 1, 1], ambient=[0, 0, 0], specular=[0, 0, 0], castshadow=False)
    R = look_at(eye, target)
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, R.ravel())
    spec.worldbody.add_camera(name="view", pos=eye, quat=q, fovy=fovy)
    model = spec.compile()
    dist = np.linalg.norm(eye - target)
    model.vis.map.znear = (dist - 1.5 * reach) / model.stat.extent
    model.vis.map.zfar = (dist + 1.5 * reach) / model.stat.extent
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, size[1], size[0])
    r.update_scene(data, camera="view")
    light = r.render()[..., 0].astype(np.float32) / 255
    r.enable_depth_rendering()
    depth = r.render().copy()
    r.disable_depth_rendering()
    r.enable_segmentation_rendering()
    seg = r.render()[..., 0].copy()  # geom id, -1 for none
    r.close()
    # Geom ids in the render scene are the mesh indices.
    return seg.astype(np.int32), depth, light, R


def project(points, eye, R, fovy, size):
    """Pixel coordinates (u right, v down) and depth along the view axis."""
    c = (points - eye) @ R  # camera frame: x right, y up, looking along -z
    z = -c[:, 2]
    f = (size[1] / 2) / math.tan(math.radians(fovy) / 2)
    u = size[0] / 2 + f * c[:, 0] / z
    v = size[1] / 2 - f * c[:, 1] / z
    return u, v, z


def crease_mask(meshes, eye, R, fovy, size, depth, crease_deg, tol, max_slope):
    """Pixels on visible creases and open edges of every mesh."""
    H, W = depth.shape
    mask = np.zeros((H, W), bool)
    far = np.where(depth > 0, depth, np.inf)
    # How much the surface's own depth changes over a pixel, so a crease is not lost where
    # its sample lands a pixel off on a steep surface. Capped, so it cannot reach through a
    # wall: blind holes in the back of a thin shell sit under a millimetre or so behind its
    # front, and must stay hidden.
    finite = np.where(np.isfinite(far), far, np.nan)
    slope = np.zeros_like(depth)
    for ax in (0, 1):
        for step in (1, -1):
            dz = np.abs(np.roll(finite, step, ax) - finite)
            slope = np.fmax(slope, np.nan_to_num(dz, nan=0.0))
    slack = tol + np.minimum(slope, max_slope)
    for _, mesh in meshes:
        sharp = mesh.face_adjacency_edges[mesh.face_adjacency_angles > math.radians(crease_deg)]
        # Edges with one face only: the mesh is open there.
        edges_sorted = np.sort(mesh.edges, axis=1)
        uniq, counts = np.unique(edges_sorted, axis=0, return_counts=True)
        edges = np.concatenate([sharp, uniq[counts == 1]])
        if len(edges) == 0:
            continue
        a, b = mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]]
        ua, va, _ = project(a, eye, R, fovy, size)
        ub, vb, _ = project(b, eye, R, fovy, size)
        n = np.maximum(2, np.ceil(2 * np.hypot(ub - ua, vb - va)).astype(int) + 1)
        idx = np.repeat(np.arange(len(edges)), n)
        t = np.concatenate([np.linspace(0, 1, k) for k in n])
        pts = a[idx] + (b[idx] - a[idx]) * t[:, None]
        u, v, z = project(pts, eye, R, fovy, size)
        ui, vi = np.round(u).astype(int), np.round(v).astype(int)
        ok = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
        ui, vi, z = ui[ok], vi[ok], z[ok]
        visible = z <= far[vi, ui] + slack[vi, ui]
        mask[vi[visible], ui[visible]] = True
    return mask


def disk(radius: float) -> np.ndarray:
    r = int(math.ceil(radius))
    y, x = np.mgrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= radius * radius


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", type=Path, default=ROBOT_DIR / "scene_allcollisions.xml")
    ap.add_argument("--out", type=Path, default=Path("docs/pictures/dukki-render.png"))
    ap.add_argument("--width", type=int, default=900, help="output width, px")
    ap.add_argument("--height", type=int, default=1000, help="output height, px")
    ap.add_argument("--supersample", type=int, default=3)
    ap.add_argument("--azimuth", type=float, default=-35.0,
                    help="camera direction from the robot, degrees about z from its front (+x)")
    ap.add_argument("--elevation", type=float, default=18.0, help="camera height, degrees")
    ap.add_argument("--fovy", type=float, default=16.0, help="vertical field of view, degrees")
    ap.add_argument("--margin", type=float, default=0.06, help="space around the robot, fraction")
    ap.add_argument("--outline", type=float, default=5.0, help="outline width, output px")
    ap.add_argument("--line", type=float, default=1.8, help="inner line width, output px")
    ap.add_argument("--crease", type=float, default=40.0, help="crease angle, degrees")
    ap.add_argument("--depth-jump", type=float, default=0.002, help="fold-behind threshold, m")
    ap.add_argument("--light", type=float, nargs=3, default=(-0.5, 0.7, 0.8),
                    help="direction the light comes from, in the camera's frame (x right, y up, "
                         "z towards the viewer)")
    ap.add_argument("--tones", type=float, nargs="+", default=(1.0, 0.86, 0.72),
                    help="fill greys, lit to shaded")
    ap.add_argument("--steps", type=float, nargs="+", default=(0.45, 0.12),
                    help="light levels (0 to 1) where each tone after the first starts")
    args = ap.parse_args()

    m, d = settle(args.scene)
    meshes, swapped = posed_meshes(m, d, args.scene)
    print(f"{len(meshes)} visual geoms, {len(swapped)} parts from the CAD exports: {', '.join(swapped)}")

    allv = np.concatenate([mesh.vertices for _, mesh in meshes])
    lo, hi = allv.min(0), allv.max(0)
    target = (lo + hi) / 2
    az, el = math.radians(args.azimuth), math.radians(args.elevation)
    direction = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])

    ss = args.supersample
    size = (args.width * ss, args.height * ss)
    # Distance that fits the robot's bounding sphere, then a pass to centre and fit tightly.
    radius = np.linalg.norm(hi - lo) / 2
    dist = radius / math.sin(math.radians(args.fovy) / 2)
    for _ in range(3):
        eye = target + direction * dist
        R = look_at(eye, target)
        u, v, _ = project(allv, eye, R, args.fovy, size)
        # Re-aim at the centre of the projected box, and scale the distance to fill the frame.
        cu, cv = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
        f = (size[1] / 2) / math.tan(math.radians(args.fovy) / 2)
        shift = R[:, 0] * (cu - size[0] / 2) / f * dist - R[:, 1] * (cv - size[1] / 2) / f * dist
        target = target + shift
        fill = max((u.max() - u.min()) / size[0], (v.max() - v.min()) / size[1])
        dist *= fill / (1 - 2 * args.margin)

    eye = target + direction * dist
    reach = np.linalg.norm(allv - target, axis=1).max()
    toward = look_at(eye, target) @ (np.asarray(args.light) / np.linalg.norm(args.light))
    seg, depth, light, R = render_buffers(meshes, eye, target, args.fovy, size, reach,
                                          light_dir=-toward, crease_deg=args.crease)
    fg = seg >= 0
    print(f"rendered {size[0]} x {size[1]}, {fg.mean():.0%} covered")

    # Inner lines from the buffers.
    lines = np.zeros_like(fg)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = seg
        b = np.roll(np.roll(seg, -dy, 0), -dx, 1)
        da = depth
        db = np.roll(np.roll(depth, -dy, 0), -dx, 1)
        both = (a >= 0) & (b >= 0)
        lines |= both & (a != b)
    # Fold-behind: depth's second difference, large where a surface jumps, small on slopes.
    dz = np.where(fg, depth, np.nan)
    lap = np.zeros_like(depth)
    for ax in (0, 1):
        second = np.abs(np.roll(dz, 1, ax) - 2 * dz + np.roll(dz, -1, ax))
        lap = np.fmax(lap, np.nan_to_num(second, nan=0.0))
    lines |= fg & (lap > args.depth_jump)
    lines |= crease_mask(meshes, eye, R, args.fovy, size, depth, args.crease,
                          tol=0.0001, max_slope=0.0003) & fg

    lines = ndimage.binary_dilation(lines, disk(args.line * ss / 2))
    # Outline straddles the silhouette.
    edge = fg ^ ndimage.binary_erosion(fg)
    outline = ndimage.binary_dilation(edge, disk(args.outline * ss / 2))

    ink = lines | outline
    alpha = (fg | outline).astype(np.float32)
    # Stepped shading: each pixel takes the tone of the band its light falls in. A median
    # filter first takes out specks where a band edge crosses fine facets.
    if len(args.steps) != len(args.tones) - 1:
        ap.error("--steps needs one fewer value than --tones")
    band = np.digitize(light, sorted(args.steps))  # 0 = darkest
    band = ndimage.median_filter(band, size=2 * ss + 1)
    tones = np.asarray(args.tones[::-1], np.float32)
    grey = np.where(ink, 0.0, tones[band]).astype(np.float32)
    rgba = np.dstack([grey, grey, grey, alpha])
    # Premultiply so the scale-down does not bleed white into the transparent edge.
    rgba[..., :3] *= rgba[..., 3:4]
    img = Image.fromarray((rgba * 255).round().astype(np.uint8), "RGBA")
    img = img.resize((args.width, args.height), Image.Resampling.BOX)
    out = np.asarray(img).astype(np.float32) / 255
    a = out[..., 3:4]
    out[..., :3] = np.where(a > 0, out[..., :3] / np.maximum(a, 1e-6), 0)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray((out.clip(0, 1) * 255).round().astype(np.uint8), "RGBA").save(args.out, optimize=True)
    print(f"saved {args.out} ({args.width} x {args.height})")


if __name__ == "__main__":
    main()
