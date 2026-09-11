"""Fusion 360 script: rebuild a MuJoCo MJCF robot as a positioned, jointed assembly.

For every MJCF body a "link" component is created under the root and placed
at the body's world pose (zero configuration). Every mesh geom becomes a
"part" component nested inside its link, holding exactly one mesh body in the
mesh's own frame; the geom's body-relative pose is carried by the occurrence
transform. Repeated parts (the same mesh asset used many times, such as a
servo or bearing) share one component by default, so redrawing that part
once updates every instance. In a parametric design each part's mesh lives
in a base feature, as the API requires. Hinge and slide joints become as-built
revolute and slider joints between the child and parent components, with
MJCF ranges applied as joint limits.

Install: Fusion > Utilities > Add-Ins > Scripts > "+" and pick this folder.
The folder must keep its name (mjcf_to_f360) and contain the .manifest file.
Run in a new, empty design. See README.md alongside this file.
"""

import importlib
import math
import os
import sys
import tempfile
import time
import traceback

import adsk.core
import adsk.fusion

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import mjcf_model  # noqa: E402
import stl_tools  # noqa: E402

# Fusion keeps imported modules alive between runs; reload so edits take effect.
importlib.reload(mjcf_model)
importlib.reload(stl_tools)

# ---------------------------------------------------------------------------
# Configuration. Edit here or leave MJCF_PATH empty to be asked for the file.
# ---------------------------------------------------------------------------

MJCF_PATH = ""                    # e.g. "/path/to/microduck_rl/.../robot_walk.xml"
INCLUDE_GEOM_GROUPS = (0, 1, 2)   # MuJoCo convention: visual meshes are group 2, collision 3
SKIP_MESHES = ()                  # mesh asset names to leave out, e.g. ("xl330",)
CREATE_JOINTS = True
CREATE_RIGID_GROUPS = True        # lock each link's parts to the link so they follow joint motion
GROUND_ROOT_BODIES = True         # ground the bodies attached directly to the world
APPLY_COLOURS = True              # copy MJCF material colours onto the mesh bodies (best effort)
RECOLOUR_ONLY = False             # True: skip import, only colour mesh bodies of an assembly built earlier
SHARE_REPEATED_PARTS = True       # one component per mesh asset, reused by every geom that shows it
VERIFY_PLACEMENT = True           # check each part's position geometrically and repair misplaced ones
LINK_NAME_FORMAT = "{body}_link"  # component name for an MJCF body ({body} = body name)
PART_NAME_FORMAT = "{mesh}"       # component name for a part ({mesh} = mesh asset name, {label} = geom label)
JOINT_AXIS_LENGTH_CM = 2.0        # length of the sketch line that defines each joint axis
BAKE_DIR = ""                     # where positioned STL copies go; empty = system temp folder
BAKE_UNITS = "mm"                 # units the baked STLs are written in and imported as
SWITCH_TO_DIRECT_MODELLING = False  # True: convert an empty parametric design to direct modelling first
PARAMETRIC_AFTER_PLACEMENT = True   # with the above: convert back to parametric once parts are placed

CM_PER_M = 100.0                  # Fusion's API works in centimetres internally


# ---------------------------------------------------------------------------

class Log:
    def __init__(self, path):
        self.path = path
        self.warnings = []
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)
        self.lines.append("WARNING: " + msg)

    def flush(self):
        try:
            with open(self.path, "w") as f:
                f.write("\n".join(self.lines) + "\n")
        except OSError:
            pass


def to_matrix3d(mat4_m, unit_scale=CM_PER_M):
    """MJCF 4x4 (metres) to a Fusion Matrix3D (centimetres)."""
    flat = mjcf_model.mat4_flat(mat4_m)
    for idx in (3, 7, 11):
        flat[idx] *= unit_scale
    m = adsk.core.Matrix3D.create()
    m.setWithArray(flat)
    return m


def point_cm(p):
    return adsk.core.Point3D.create(p[0] * CM_PER_M, p[1] * CM_PER_M, p[2] * CM_PER_M)


def choose_mjcf(ui):
    if MJCF_PATH and os.path.isfile(MJCF_PATH):
        return MJCF_PATH
    dlg = ui.createFileDialog()
    dlg.title = "Select the MJCF robot file"
    dlg.filter = "MJCF files (*.xml);;All files (*.*)"
    dlg.isMultiSelectEnabled = False
    if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.filename


def mesh_units():
    unit_name = {"mm": "MillimeterMeshUnit", "cm": "CentimeterMeshUnit", "m": "MeterMeshUnit"}[BAKE_UNITS]
    return getattr(adsk.fusion.MeshUnits, unit_name)


def add_mesh_body(app, comp, path, base_feature, log):
    """Add an STL to a component and return the new mesh bodies.

    Preferred route is MeshBodies.add. In a parametric design that call must
    happen while a base feature is being edited, which the caller arranges.
    The import manager is kept as a fallback for older Fusion builds.
    """
    before = comp.meshBodies.count
    try:
        if base_feature is not None:
            comp.meshBodies.add(path, mesh_units(), base_feature)
        else:
            comp.meshBodies.add(path, mesh_units())
    except Exception as exc:  # noqa: BLE001
        log.warn("meshBodies.add failed for %s (%s); trying the import manager" % (os.path.basename(path), exc))
        opts = app.importManager.createSTLImportOptions(path)
        opts.isViewFit = False
        for attr in ("unitType", "units"):
            if hasattr(opts, attr):
                try:
                    setattr(opts, attr, mesh_units())
                    break
                except Exception:  # noqa: BLE001
                    continue
        app.importManager.importToTarget(opts, comp)
    return [comp.meshBodies.item(k) for k in range(before, comp.meshBodies.count)]


class AppearanceCache:
    """Creates one design appearance per MJCF colour, copied from a library appearance.

    Any opaque appearance with an editable colour works as the base. Names in
    the appearance libraries vary between Fusion versions and languages, so the
    search is by loose match first, then by any appearance exposing a colour.
    """

    PREFERRED = ("plastic - matte", "plastic - glossy", "paint - enamel", "plastic", "paint", "abs")
    COLOUR_IDS = ("opaque_albedo", "generic_diffuse", "surface_albedo")

    def __init__(self, app, design, log):
        self.design = design
        self.log = log
        self.cache = {}
        self.failed = False
        self.base, self.colour_id = self._find_base(app, design, log)
        if self.base is None:
            self.failed = True
            log.warn("no base appearance found in the material libraries; colours skipped")
        else:
            log.info("appearance base: %s (colour property %s)" % (self.base.name, self.colour_id))

    @classmethod
    def _colour_property(cls, appearance):
        try:
            props = appearance.appearanceProperties
        except Exception:  # noqa: BLE001
            return None
        for pid in cls.COLOUR_IDS:
            try:
                prop = props.itemById(pid)
            except Exception:  # noqa: BLE001
                prop = None
            if prop is not None and prop.objectType == adsk.core.ColorProperty.classType():
                return pid
        try:
            for i in range(props.count):
                prop = props.item(i)
                if prop.objectType == adsk.core.ColorProperty.classType() and not prop.isReadOnly:
                    return prop.id
        except Exception:  # noqa: BLE001
            pass
        return None

    def _find_base(self, app, design, log):
        sources = []
        try:
            for i in range(app.materialLibraries.count):
                lib = app.materialLibraries.item(i)
                try:
                    count = lib.appearances.count
                except Exception as exc:  # noqa: BLE001
                    log.info("library %s has no readable appearances (%s)" % (lib.name, exc))
                    continue
                log.info("library %s: %d appearances" % (lib.name, count))
                sources.append(lib.appearances)
        except Exception as exc:  # noqa: BLE001
            log.warn("could not enumerate material libraries: %s" % exc)
        try:
            if design.appearances.count:
                sources.append(design.appearances)
        except Exception:  # noqa: BLE001
            pass

        fallback = None
        for appearances in sources:
            for k in range(appearances.count):
                try:
                    appearance = appearances.item(k)
                    name = appearance.name.lower()
                except Exception:  # noqa: BLE001
                    continue
                colour_id = self._colour_property(appearance)
                if colour_id is None:
                    continue
                if any(name.startswith(pref) for pref in self.PREFERRED):
                    return appearance, colour_id
                if fallback is None:
                    fallback = (appearance, colour_id)
        return fallback if fallback else (None, None)

    def get(self, rgba):
        if self.failed:
            return None
        key = tuple(int(round(c * 255)) for c in rgba[:3])
        if key in self.cache:
            return self.cache[key]
        name = "mjcf_%02x%02x%02x" % key
        try:
            appearance = self.design.appearances.itemByName(name)
            if appearance is None:
                appearance = self.design.appearances.addByCopy(self.base, name)
                prop = appearance.appearanceProperties.itemById(self.colour_id)
                prop.value = adsk.core.Color.create(key[0], key[1], key[2], 255)
            self.cache[key] = appearance
            return appearance
        except Exception as exc:  # noqa: BLE001
            self.log.warn("appearance %s failed: %s" % (name, exc))
            self.cache[key] = None
            return None


def build_components(model, root, log, progress):
    """Create one link component per body at its world pose. Returns {body name: occurrence}."""
    occurrences = {}
    for body in model.articulated_bodies():
        occ = root.occurrences.addNewComponent(to_matrix3d(body.world))
        occ.component.name = LINK_NAME_FORMAT.format(body=body.name)
        occ.component.description = "MJCF body %s (parent %s)" % (body.name, body.parent.name)
        if GROUND_ROOT_BODIES and body.parent is model.world:
            try:
                occ.isGrounded = True
            except Exception as exc:  # noqa: BLE001
                log.warn("could not ground %s: %s" % (body.name, exc))
        occurrences[body.name] = occ
        log.info("link %s at %s" % (occ.component.name, mjcf_model.mat4_pos(body.world)))
    return occurrences


def matrices_close(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a.asArray(), b.asArray()))


class Placement:
    """A part occurrence waiting to be moved to its world pose."""

    def __init__(self, part_occ, link_occ, geom, comp, name):
        self.part_occ = part_occ
        self.link_occ = link_occ
        self.geom = geom
        self.comp = comp
        self.name = name
        self.geom_mesh = None
        self.world4 = mjcf_model.mat4_mul(geom.body.world, mjcf_model.mat4_from(geom.rot, geom.pos))
        self.world = to_matrix3d(self.world4)
        try:
            self.proxy = part_occ.createForAssemblyContext(link_occ)
        except Exception:  # noqa: BLE001
            self.proxy = None

    def describe(self):
        return "%s in %s" % (self.name, self.geom.body.name)

    def transforms_text(self):
        """Translations (cm) of the native and proxy transforms, for the log."""
        def trans(obj):
            try:
                a = obj.transform2.asArray()
                return [round(a[3], 2), round(a[7], 2), round(a[11], 2)]
            except Exception as exc:  # noqa: BLE001
                return "n/a (%s)" % exc
        wa = self.world.asArray()
        return "expected world %s, native transform2 %s, proxy transform2 %s" % (
            [round(wa[3], 2), round(wa[7], 2), round(wa[11], 2)], trans(self.part_occ),
            trans(self.proxy) if self.proxy is not None else "none")


def place_all(root, design, placements, log):
    """Move every part occurrence to its world pose in one root-relative call.

    Component.transformOccurrences works on the flattened assembly, so every
    transform is relative to the root regardless of nesting. Per-occurrence
    transform properties proved to behave differently for the first and for
    repeated occurrences of a shared component, so they are only a fallback.
    """
    todo = [p for p in placements if p.proxy is not None]
    for p in placements:
        if p.proxy is None:
            log.warn("no root-context proxy for %s; not positioned" % p.describe())
    if not todo:
        return
    done = False
    for wrap in (list, _object_collection):
        try:
            ok = root.transformOccurrences(wrap([p.proxy for p in todo]), wrap([p.world for p in todo]), True)
            log.info("transformOccurrences on %d parts returned %s" % (len(todo), ok))
            done = bool(ok)
            break
        except Exception as exc:  # noqa: BLE001
            log.info("transformOccurrences with %s failed: %s" % (wrap.__name__, exc))
    if not done:
        log.warn("bulk placement unavailable; setting occurrence transforms one by one")
        for p in todo:
            try:
                p.proxy.transform2 = p.world
            except Exception as exc:  # noqa: BLE001
                log.warn("transform on %s failed: %s" % (p.describe(), exc))
    capture_position(design, log)


def _object_collection(items):
    coll = adsk.core.ObjectCollection.create()
    for item in items:
        coll.add(item)
    return coll


class PlacementChecker:
    """Compares a part's root-context bounding box in Fusion with the expected one.

    Expected boxes come from the baked STL vertices under the world pose, so
    the check does not depend on how Fusion interprets occurrence transforms.
    """

    def __init__(self, bake_dir, log):
        self.bake_dir = bake_dir
        self.log = log
        self.unit_scale = stl_tools.UNIT_SCALES[BAKE_UNITS]
        self.verts = {}

    def _vertices(self, mesh):
        path = stl_tools.bake_mesh(mesh, self.bake_dir, self.unit_scale)
        if path not in self.verts:
            pts = set()
            for tri in stl_tools.read_stl(path):
                pts.update(tri)
            self.verts[path] = [[c / self.unit_scale for c in v] for v in pts]
        return self.verts[path]

    def expected_box(self, p):
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for v in self._vertices(p.geom_mesh):
            w = mjcf_model.mat4_apply_point(p.world4, v)
            for k in range(3):
                c = w[k] * CM_PER_M
                lo[k] = min(lo[k], c)
                hi[k] = max(hi[k], c)
        return lo, hi

    def error_cm(self, p):
        """Largest corner deviation in cm, or None when Fusion cannot report a box."""
        if p.proxy is None:
            return None
        try:
            mesh = p.comp.meshBodies.item(0)
            box = mesh.createForAssemblyContext(p.proxy).boundingBox
            got_min, got_max = box.minPoint.asArray(), box.maxPoint.asArray()
        except Exception as exc:  # noqa: BLE001
            self.log.info("verify %s: bounding box unavailable (%s)" % (p.describe(), exc))
            return None
        lo, hi = self.expected_box(p)
        return max(max(abs(a - b) for a, b in zip(lo, got_min)), max(abs(a - b) for a, b in zip(hi, got_max)))


def verify_and_repair(root, design, placements, bake_dir, log, tol_cm=0.05):
    """Check every part; for misplaced ones try alternative placement strategies until one verifies.

    Strategies, in order:
      1. set the world pose on the root-context proxy
      2. set the link-relative pose on the native occurrence
      3. recreate the occurrence in the link with the link-relative pose and leave it alone
      4. recreate the occurrence in the link with the world pose
      5. recreate the occurrence directly under the root with the world pose (loses nesting)
    The log records which strategy fixed each part so the default path can be
    tightened once the behaviour is understood.
    """
    checker = PlacementChecker(bake_dir, log)
    bad = []
    for p in placements:
        err = checker.error_cm(p)
        if err is None:
            continue
        if err > tol_cm:
            bad.append(p)
            lo, _ = checker.expected_box(p)
            log.warn("verify %s: OFF by %.2f cm after bulk placement (expected min %s)"
                     % (p.describe(), err, [round(c, 2) for c in lo]))
            log.info("   %s" % p.transforms_text())
        else:
            log.info("verify %s: ok (%.3f cm)" % (p.describe(), err))
    if not bad:
        return 0

    log.info("repairing %d misplaced parts" % len(bad))
    still_bad = 0
    for p in bad:
        fixed_by = None
        for name, strategy in (("proxy world pose", _fix_proxy_world),
                               ("native local pose", _fix_native_local),
                               ("recreate in link (local)", _fix_recreate_link_local),
                               ("recreate in link (world)", _fix_recreate_link_world),
                               ("recreate under root (world)", _fix_recreate_root_world)):
            try:
                strategy(root, p)
            except Exception as exc:  # noqa: BLE001
                log.info("repair %s via %s raised %s" % (p.describe(), name, exc))
                continue
            err = checker.error_cm(p)
            if err is not None and err <= tol_cm:
                fixed_by = name
                break
            log.info("repair %s via %s: still off by %s cm" % (p.describe(), name,
                                                            "?" if err is None else "%.2f" % err))
        if fixed_by:
            log.warn("repaired %s via %s" % (p.describe(), fixed_by))
            log.info("   %s" % p.transforms_text())
        else:
            still_bad += 1
            log.warn("could not place %s correctly" % p.describe())
    capture_position(design, log)
    for p in bad:
        err = checker.error_cm(p)
        if err is not None and err > tol_cm:
            log.warn("after capture %s: OFF by %.2f cm again (moved by the position capture)" % (p.describe(), err))
            log.info("   %s" % p.transforms_text())
        elif err is not None:
            log.info("after capture %s: ok" % p.describe())
    if still_bad:
        log.warn("%d of %d parts are not where the MJCF puts them" % (still_bad, len(placements)))
    return still_bad


def final_verify(placements, bake_dir, log, tol_cm=0.05):
    """Last check after joints and captures, so the log reflects what is on screen."""
    checker = PlacementChecker(bake_dir, log)
    off = []
    for p in placements:
        err = checker.error_cm(p)
        if err is not None and err > tol_cm:
            off.append("%s (%.2f cm)" % (p.describe(), err))
    if off:
        log.warn("final check: %d parts misplaced: %s" % (len(off), "; ".join(off)))
    else:
        log.info("final check: all %d parts in place" % len(placements))
    return len(off)


def _fix_proxy_world(root, p):
    p.proxy.transform2 = p.world


def _fix_native_local(root, p):
    p.part_occ.transform2 = local_matrix3d(p.geom)


def _recreate(p, occurrences, matrix, nested):
    old = p.part_occ
    new = occurrences.addExistingComponent(p.comp, matrix)
    try:
        old.deleteMe()
    except Exception:  # noqa: BLE001
        pass
    p.part_occ = new
    p.proxy = new.createForAssemblyContext(p.link_occ) if nested else new


def _fix_recreate_link_local(root, p):
    _recreate(p, p.link_occ.component.occurrences, local_matrix3d(p.geom), True)


def _fix_recreate_link_world(root, p):
    _recreate(p, p.link_occ.component.occurrences, p.world, True)


def _fix_recreate_root_world(root, p):
    _recreate(p, root.occurrences, p.world, False)


def capture_position(design, log):
    """Record pending occurrence moves in a parametric timeline so they cannot revert."""
    try:
        if design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
            return
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
            log.info("captured component positions")
    except Exception as exc:  # noqa: BLE001
        log.warn("capture position failed: %s" % exc)


def local_matrix3d(geom):
    return to_matrix3d(mjcf_model.mat4_from(geom.rot, geom.pos))


def create_part_component(app, design, link_comp, name, mesh, geom, bake_dir, unit_scale, cache, log, rgba,
                          appearances):
    """Create a new component under link_comp holding one mesh body. Returns (occurrence, component)."""
    parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    occ = link_comp.occurrences.addNewComponent(local_matrix3d(geom))
    comp = occ.component
    comp.name = name
    comp.description = "MJCF mesh %s (%s)" % (mesh.name, os.path.basename(mesh.path))
    baked = stl_tools.bake_mesh(mesh, bake_dir, unit_scale, cache)
    base_feature = None
    if parametric:
        base_feature = comp.features.baseFeatures.add()
        base_feature.name = "mesh_%s" % mesh.name
        base_feature.startEdit()
    try:
        new_bodies = add_mesh_body(app, comp, baked, base_feature, log)
    finally:
        if base_feature is not None:
            base_feature.finishEdit()
    if not new_bodies:
        log.warn("Fusion did not create a mesh body for %s" % baked)
    for k, mesh_body in enumerate(new_bodies):
        try:
            mesh_body.name = name if k == 0 else "%s_%d" % (name, k)
        except Exception:  # noqa: BLE001
            pass
        if appearances is not None:
            appearance = appearances.get(rgba)
            if appearance is not None:
                try:
                    mesh_body.appearance = appearance
                except Exception as exc:  # noqa: BLE001
                    log.warn("appearance on %s failed: %s" % (name, exc))
    return occ, comp


def import_meshes(app, design, model, occurrences, bake_dir, log, progress, appearances):
    """Create part components inside the links. Returns (parts created, occurrences placed)."""
    unit_scale = stl_tools.UNIT_SCALES[BAKE_UNITS]
    cache = {}
    parts = {}          # share key -> component
    part_colours = {}   # share key -> rgba first seen, to report conflicts
    created = placed = 0
    placements = []
    for body in model.articulated_bodies():
        link_occ = occurrences[body.name]
        link_comp = link_occ.component
        for i, geom in enumerate(body.mesh_geoms(set(INCLUDE_GEOM_GROUPS))):
            label = model.geom_label(geom, i)
            if progress.wasCancelled:
                raise KeyboardInterrupt
            progress.message = "Importing %s / %s" % (body.name, label)
            progress.progressValue += 1
            if geom.mesh in SKIP_MESHES:
                log.info("skipped %s (in SKIP_MESHES)" % label)
                continue
            if geom.mesh not in model.meshes:
                log.warn("geom %s references unknown mesh %s" % (label, geom.mesh))
                continue
            mesh = model.meshes[geom.mesh]
            if not os.path.isfile(mesh.path):
                log.warn("mesh file missing: %s" % mesh.path)
                continue
            rgba = model.geom_rgba(geom)
            key = geom.mesh if SHARE_REPEATED_PARTS else (body.name, label)
            name = PART_NAME_FORMAT.format(mesh=geom.mesh, label=label)
            try:
                if key in parts:
                    comp = parts[key]
                    part_occ = link_comp.occurrences.addExistingComponent(comp, local_matrix3d(geom))
                    if part_colours.get(key) != rgba:
                        log.info("part %s reused in %s with a different MJCF colour; first colour kept"
                                 % (name, body.name))
                else:
                    part_occ, comp = create_part_component(app, design, link_comp, name, mesh, geom, bake_dir,
                                                           unit_scale, cache, log, rgba, appearances)
                    parts[key] = comp
                    part_colours[key] = rgba
                    created += 1
                placement = Placement(part_occ, link_occ, geom, comp, name)
                placement.geom_mesh = mesh
                placements.append(placement)
                placed += 1
                log.info("created %s in %s (local pos %s)" % (name, body.name, geom.pos))
            except Exception as exc:  # noqa: BLE001
                log.warn("part %s in %s failed: %s\n%s" % (label, body.name, exc, traceback.format_exc()))
    progress.message = "Positioning parts"
    place_all(design.rootComponent, design, placements, log)
    if VERIFY_PLACEMENT:
        progress.message = "Verifying part positions"
        verify_and_repair(design.rootComponent, design, placements, bake_dir, log)
    return created, placed, placements


def create_joints(model, design, root, occurrences, log, progress):
    """Create Fusion joints for every hinge and slide.

    As-built joints are the natural match (they keep the current positions)
    but Fusion only offers them in parametric designs. In a direct design the
    script uses regular joints instead, with the joint geometry drawn at the
    same world location in both links so that nothing moves when the joint is
    applied.
    """
    parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    log.info("creating joints as %s" % ("as-built joints" if parametric else "regular joints (direct design)"))
    created = 0
    for joint in model.joints:
        if joint.jtype == "free":
            continue
        body = joint.body
        if body.parent is model.world:
            log.info("joint %s attaches %s to the world; body left grounded instead" % (joint.name, body.name))
            continue
        if joint.jtype not in ("hinge", "slide"):
            log.warn("joint %s is of type %s; only hinge and slide are created" % (joint.name, joint.jtype))
            continue
        progress.message = "Joint %s" % joint.name
        progress.progressValue += 1
        child_occ = occurrences[body.name]
        parent_occ = occurrences[body.parent.name]
        try:
            if parametric:
                _create_as_built_joint(joint, root, child_occ, parent_occ, log)
            else:
                _create_regular_joint(joint, root, child_occ, parent_occ, log)
            created += 1
        except Exception as exc:  # noqa: BLE001
            log.warn("joint %s failed: %s\n%s" % (joint.name, exc, traceback.format_exc()))
    return created


def _axis_line(occ, name, pos_m, axis, log):
    """Draw a construction line in occ's component from pos along axis; return its root-context proxy."""
    comp = occ.component
    axis = mjcf_model.vec_normalise(axis)
    p0 = point_cm(pos_m)
    p1 = adsk.core.Point3D.create(p0.x + axis[0] * JOINT_AXIS_LENGTH_CM,
                                  p0.y + axis[1] * JOINT_AXIS_LENGTH_CM,
                                  p0.z + axis[2] * JOINT_AXIS_LENGTH_CM)
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = name
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(sketch.modelToSketchSpace(p0),
                                                          sketch.modelToSketchSpace(p1))
    line.isConstruction = True
    return line.createForAssemblyContext(occ)


def _joint_frame_in_parent(joint):
    """Joint origin and axis expressed in the parent body's frame."""
    child, parent = joint.body, joint.body.parent
    origin_w = mjcf_model.mat4_apply_point(child.world, joint.pos)
    axis_w = mjcf_model.mat4_apply_vector(child.world, joint.axis)
    rot_p_t = mjcf_model.mat3_transpose(mjcf_model.mat4_rot(parent.world))
    pos_p = mjcf_model.mat4_pos(parent.world)
    origin_p = mjcf_model.mat3_apply(rot_p_t, [origin_w[i] - pos_p[i] for i in range(3)])
    axis_p = mjcf_model.mat3_apply(rot_p_t, axis_w)
    return origin_p, axis_p


def _apply_motion(joint_input, joint, axis_entity):
    custom = adsk.fusion.JointDirections.CustomJointDirection
    if joint.jtype == "hinge":
        joint_input.setAsRevoluteJointMotion(custom, axis_entity)
    else:
        joint_input.setAsSliderJointMotion(custom, axis_entity)


def _apply_limits(fusion_joint, joint, log):
    if joint.name:
        fusion_joint.name = joint.name
    if joint.range is None:
        return
    try:
        motion = fusion_joint.jointMotion
        limits = motion.rotationLimits if joint.jtype == "hinge" else motion.slideLimits
        lo, hi = joint.range
        if joint.jtype == "slide":
            lo, hi = lo * CM_PER_M, hi * CM_PER_M
        limits.isRestValueEnabled = True
        limits.restValue = 0.0
        limits.isMinimumValueEnabled = True
        limits.minimumValue = lo
        limits.isMaximumValueEnabled = True
        limits.maximumValue = hi
    except Exception as exc:  # noqa: BLE001
        log.warn("limits on joint %s not applied: %s" % (joint.name, exc))


def _create_as_built_joint(joint, root, child_occ, parent_occ, log):
    line_proxy = _axis_line(child_occ, "axis_%s" % (joint.name or "joint"), joint.pos, joint.axis, log)
    geometry = adsk.fusion.JointGeometry.createByCurve(line_proxy, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
    joint_input = root.asBuiltJoints.createInput(child_occ, parent_occ, geometry)
    _apply_motion(joint_input, joint, line_proxy)
    fusion_joint = root.asBuiltJoints.add(joint_input)
    _apply_limits(fusion_joint, joint, log)
    log.info("as-built joint %s: %s -> %s" % (joint.name, child_occ.component.name, parent_occ.component.name))


def _create_regular_joint(joint, root, child_occ, parent_occ, log):
    child_line = _axis_line(child_occ, "axis_%s" % (joint.name or "joint"), joint.pos, joint.axis, log)
    origin_p, axis_p = _joint_frame_in_parent(joint)
    parent_line = _axis_line(parent_occ, "axis_%s_parent" % (joint.name or "joint"), origin_p, axis_p, log)
    start = adsk.fusion.JointKeyPointTypes.StartKeyPoint
    geo_child = adsk.fusion.JointGeometry.createByCurve(child_line, start)
    geo_parent = adsk.fusion.JointGeometry.createByCurve(parent_line, start)
    joint_input = root.joints.createInput(geo_child, geo_parent)
    _apply_motion(joint_input, joint, child_line)
    fusion_joint = root.joints.add(joint_input)
    _apply_limits(fusion_joint, joint, log)
    log.info("regular joint %s: %s -> %s" % (joint.name, child_occ.component.name, parent_occ.component.name))


def create_rigid_groups(root, occurrences, placements, log):
    """One rigid group per link, so its parts move with it when a joint is driven.

    Fusion's assembly solver works on the flattened structure: nesting alone
    does not bind a part to its link. Rigid groups do, without adding a joint
    per part.
    """
    by_link = {}
    for p in placements:
        if p.proxy is not None:
            by_link.setdefault(p.geom.body.name, []).append(p.proxy)
    created = 0
    for body_name, link_occ in occurrences.items():
        parts = by_link.get(body_name, [])
        if not parts:
            continue
        group = None
        try:
            coll = _object_collection([link_occ])
            group = root.rigidGroups.add(coll, True)
        except Exception as exc:  # noqa: BLE001
            log.info("rigid group for %s with children failed (%s); listing the parts explicitly" % (body_name, exc))
            try:
                group = root.rigidGroups.add(_object_collection([link_occ] + parts), False)
            except Exception as exc2:  # noqa: BLE001
                log.warn("rigid group for %s failed: %s" % (body_name, exc2))
        if group is not None:
            try:
                group.name = "%s_rigid" % link_occ.component.name
            except Exception:  # noqa: BLE001
                pass
            created += 1
            log.info("rigid group %s: %d parts" % (link_occ.component.name, len(parts)))
    return created


def convert_to_parametric(design, log):
    """Turn a direct design parametric so as-built joints and a timeline are available."""
    try:
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        log.info("design converted to parametric before creating joints")
        return True
    except Exception as exc:  # noqa: BLE001
        log.warn("could not convert to parametric (%s); regular joints will be used" % exc)
        return False


def recolour_existing(model, root, log, appearances):
    """Apply MJCF colours to part components created by an earlier run of this script."""
    wanted = {}
    for body in model.articulated_bodies():
        for i, geom in enumerate(body.mesh_geoms(set(INCLUDE_GEOM_GROUPS))):
            name = PART_NAME_FORMAT.format(mesh=geom.mesh, label=model.geom_label(geom, i))
            wanted.setdefault(name, model.geom_rgba(geom))
    coloured, seen = 0, set()
    for i in range(root.allOccurrences.count):
        occ = root.allOccurrences.item(i)
        comp = occ.component
        if comp.name not in wanted or comp.name in seen:
            continue
        seen.add(comp.name)
        appearance = appearances.get(wanted[comp.name])
        if appearance is None:
            continue
        for k in range(comp.meshBodies.count):
            try:
                comp.meshBodies.item(k).appearance = appearance
                coloured += 1
            except Exception as exc:  # noqa: BLE001
                log.warn("appearance on %s failed: %s" % (comp.name, exc))
    for name in set(wanted) - seen:
        log.warn("no part component named %s in this design" % name)
    return coloured


def run(context):  # noqa: ARG001  (Fusion entry point)
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox("Open a design first (a new, empty design is recommended).")
            return

        path = choose_mjcf(ui)
        if not path:
            return

        model = mjcf_model.MjcfModel(path)
        bake_dir = BAKE_DIR or os.path.join(tempfile.gettempdir(), "mjcf_to_f360", model.name)
        os.makedirs(bake_dir, exist_ok=True)
        log = Log(os.path.join(bake_dir, "import_%s.log" % time.strftime("%Y%m%d_%H%M%S")))
        log.info("source %s" % model.path)
        log.info(model.summary())
        for w in model.warnings:
            log.warn(w)

        if RECOLOUR_ONLY:
            appearances = AppearanceCache(app, design, log)
            coloured = recolour_existing(model, design.rootComponent, log, appearances)
            log.flush()
            summary = "Recoloured %d mesh bodies. %d warning(s).\nLog: %s" % (coloured, len(log.warnings), log.path)
            if log.warnings:
                summary += "\n\nFirst warnings:\n" + "\n".join(log.warnings[:8])
            ui.messageBox(summary)
            return

        groups = set(INCLUDE_GEOM_GROUPS)
        n_geoms = sum(len(b.mesh_geoms(groups)) for b in model.articulated_bodies())
        n_joints = sum(1 for j in model.joints if j.jtype in ("hinge", "slide"))
        total = max(1, n_geoms + (n_joints if CREATE_JOINTS else 0))

        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.show("MJCF to Fusion", "Creating components", 0, total, 0)

        if SWITCH_TO_DIRECT_MODELLING and design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
            try:
                design.designType = adsk.fusion.DesignTypes.DirectDesignType
                log.info("design switched to direct modelling")
            except Exception as exc:  # noqa: BLE001
                log.warn("could not switch to direct modelling: %s" % exc)

        root = design.rootComponent
        design.activateRootComponent()
        occurrences = build_components(model, root, log, progress)
        appearances = AppearanceCache(app, design, log) if APPLY_COLOURS else None
        created, placed, placements = import_meshes(app, design, model, occurrences, bake_dir, log, progress,
                                                    appearances)
        if (SWITCH_TO_DIRECT_MODELLING and PARAMETRIC_AFTER_PLACEMENT
                and design.designType != adsk.fusion.DesignTypes.ParametricDesignType):
            convert_to_parametric(design, log)
            if VERIFY_PLACEMENT:
                log.info("re-checking part positions after the conversion")
                final_verify(placements, bake_dir, log)
        rigid = create_rigid_groups(root, occurrences, placements, log) if CREATE_RIGID_GROUPS else 0
        joints = create_joints(model, design, root, occurrences, log, progress) if CREATE_JOINTS else 0
        capture_position(design, log)
        misplaced = final_verify(placements, bake_dir, log) if VERIFY_PLACEMENT else 0
        progress.hide()

        log.flush()
        summary = ("Imported %s\n\n%d links, %d part components placed %d times, %d rigid groups, %d joints.\n"
                   "%d part(s) misplaced at the end, %d warning(s).\n\nBaked meshes and log: %s"
                   % (model.name, len(occurrences), created, placed, rigid, joints, misplaced, len(log.warnings),
                      bake_dir))
        if log.warnings:
            summary += "\n\nFirst warnings:\n" + "\n".join(log.warnings[:8])
        ui.messageBox(summary)
        app.activeViewport.fit()
    except KeyboardInterrupt:
        if ui:
            ui.messageBox("Import cancelled.")
    except Exception:  # noqa: BLE001
        if ui:
            ui.messageBox("mjcf_to_f360 failed:\n{}".format(traceback.format_exc()))


def stop(context):  # noqa: ARG001
    pass
