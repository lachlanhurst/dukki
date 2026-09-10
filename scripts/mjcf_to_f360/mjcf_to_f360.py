"""Fusion 360 script: rebuild a MuJoCo MJCF robot as a positioned, jointed assembly.

For every MJCF body a component is created under the root and placed at the
body's world pose (zero configuration). Every mesh geom is added to its
body's component as a mesh body, with the geom's body-relative pose baked
into a temporary STL copy beforehand. In a parametric design the meshes of
each body live in one base feature, as the API requires. Hinge and slide joints become as-built
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
GROUND_ROOT_BODIES = True         # ground the bodies attached directly to the world
APPLY_COLOURS = True              # copy MJCF material colours onto the mesh bodies (best effort)
JOINT_AXIS_LENGTH_CM = 2.0        # length of the sketch line that defines each joint axis
BAKE_DIR = ""                     # where positioned STL copies go; empty = system temp folder
BAKE_UNITS = "mm"                 # units the baked STLs are written in and imported as
SWITCH_TO_DIRECT_MODELLING = False  # True: convert an empty parametric design to direct modelling first

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
    """Creates one design appearance per MJCF colour, copied from a library plastic."""

    BASE_CANDIDATES = ("Plastic - Matte (Black)", "Plastic - Glossy (Black)", "Paint - Enamel Glossy (Black)",
                       "ABS (White)")

    def __init__(self, app, design, log):
        self.design = design
        self.log = log
        self.cache = {}
        self.base = None
        self.failed = False
        try:
            for i in range(app.materialLibraries.count):
                lib = app.materialLibraries.item(i)
                for name in self.BASE_CANDIDATES:
                    base = lib.appearances.itemByName(name)
                    if base is not None:
                        self.base = base
                        break
                if self.base is not None:
                    break
        except Exception:  # noqa: BLE001
            self.base = None
        if self.base is None:
            self.failed = True
            log.warn("no base appearance found in the material libraries; colours skipped")

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
                prop = appearance.appearanceProperties.itemById("opaque_albedo")
                if prop is None:
                    for i in range(appearance.appearanceProperties.count):
                        p = appearance.appearanceProperties.item(i)
                        if p.objectType == adsk.core.ColorProperty.classType():
                            prop = p
                            break
                if prop is not None:
                    prop.value = adsk.core.Color.create(key[0], key[1], key[2], 255)
            self.cache[key] = appearance
            return appearance
        except Exception as exc:  # noqa: BLE001
            self.log.warn("appearance %s failed: %s" % (name, exc))
            self.cache[key] = None
            return None


def build_components(model, root, log, progress):
    """Create one component per body at its world pose. Returns {body name: occurrence}."""
    occurrences = {}
    for body in model.articulated_bodies():
        occ = root.occurrences.addNewComponent(to_matrix3d(body.world))
        occ.component.name = body.name
        occ.component.description = "MJCF body %s (parent %s)" % (body.name, body.parent.name)
        if GROUND_ROOT_BODIES and body.parent is model.world:
            try:
                occ.isGrounded = True
            except Exception as exc:  # noqa: BLE001
                log.warn("could not ground %s: %s" % (body.name, exc))
        occurrences[body.name] = occ
        log.info("component %s at %s" % (body.name, mjcf_model.mat4_pos(body.world)))
    return occurrences


def import_meshes(app, design, model, occurrences, bake_dir, log, progress, appearances):
    parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    unit_scale = stl_tools.UNIT_SCALES[BAKE_UNITS]
    cache = {}
    imported = 0
    for body in model.articulated_bodies():
        comp = occurrences[body.name].component
        geoms = body.mesh_geoms(set(INCLUDE_GEOM_GROUPS))
        if not geoms:
            continue
        base_feature = None
        if parametric:
            base_feature = comp.features.baseFeatures.add()
            base_feature.name = "meshes_%s" % body.name
            base_feature.startEdit()
        try:
            for i, geom in enumerate(geoms):
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
                try:
                    baked = stl_tools.bake_geom(model, geom, bake_dir, unit_scale, cache)
                except Exception as exc:  # noqa: BLE001
                    log.warn("baking %s failed: %s" % (label, exc))
                    continue
                new_bodies = add_mesh_body(app, comp, baked, base_feature, log)
                if not new_bodies:
                    log.warn("Fusion did not create a mesh body for %s" % baked)
                    continue
                for k, mesh_body in enumerate(new_bodies):
                    try:
                        mesh_body.name = label if k == 0 else "%s_%d" % (label, k)
                    except Exception:  # noqa: BLE001
                        pass
                    if appearances is not None:
                        appearance = appearances.get(model.geom_rgba(geom))
                        if appearance is not None:
                            try:
                                mesh_body.appearance = appearance
                            except Exception as exc:  # noqa: BLE001
                                log.warn("appearance on %s failed: %s" % (label, exc))
                imported += 1
                log.info("imported %s into %s from %s" % (label, body.name, os.path.basename(baked)))
        finally:
            if base_feature is not None:
                base_feature.finishEdit()
    return imported


def create_joints(model, root, occurrences, log, progress):
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
            _create_one_joint(joint, root, child_occ, parent_occ, log)
            created += 1
        except Exception as exc:  # noqa: BLE001
            log.warn("joint %s failed: %s\n%s" % (joint.name, exc, traceback.format_exc()))
    return created


def _create_one_joint(joint, root, child_occ, parent_occ, log):
    comp = child_occ.component
    axis = mjcf_model.vec_normalise(joint.axis)
    p0 = point_cm(joint.pos)
    p1 = adsk.core.Point3D.create(p0.x + axis[0] * JOINT_AXIS_LENGTH_CM,
                                  p0.y + axis[1] * JOINT_AXIS_LENGTH_CM,
                                  p0.z + axis[2] * JOINT_AXIS_LENGTH_CM)

    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.name = "axis_%s" % (joint.name or "joint")
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(sketch.modelToSketchSpace(p0),
                                                          sketch.modelToSketchSpace(p1))
    line.isConstruction = True
    line_proxy = line.createForAssemblyContext(child_occ)

    geometry = adsk.fusion.JointGeometry.createByCurve(line_proxy, adsk.fusion.JointKeyPointTypes.StartKeyPoint)
    joint_input = root.asBuiltJoints.createInput(child_occ, parent_occ, geometry)
    custom = adsk.fusion.JointDirections.CustomJointDirection
    if joint.jtype == "hinge":
        joint_input.setAsRevoluteJointMotion(custom, line_proxy)
    else:
        joint_input.setAsSliderJointMotion(custom, line_proxy)
    fusion_joint = root.asBuiltJoints.add(joint_input)
    if joint.name:
        fusion_joint.name = joint.name

    if joint.range is not None:
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
    log.info("joint %s: %s -> %s about %s" % (joint.name, child_occ.component.name,
                                              parent_occ.component.name, axis))


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
        imported = import_meshes(app, design, model, occurrences, bake_dir, log, progress, appearances)
        joints = create_joints(model, root, occurrences, log, progress) if CREATE_JOINTS else 0
        progress.hide()

        log.flush()
        summary = ("Imported %s\n\n%d components, %d mesh bodies, %d joints.\n%d warning(s).\n\n"
                   "Baked meshes and log: %s" % (model.name, len(occurrences), imported, joints,
                                                 len(log.warnings), bake_dir))
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
