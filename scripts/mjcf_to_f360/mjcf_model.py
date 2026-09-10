"""Minimal MJCF (MuJoCo XML) reader.

Standard library only, so it can run inside Fusion 360's bundled Python as
well as on a normal machine. It covers what is needed to rebuild a robot as a
positioned assembly: bodies, joints, mesh geoms, mesh assets, materials,
default classes and <include> files. Everything is evaluated at the zero
pose (all joint angles zero).

Conventions follow MuJoCo: lengths in metres, quaternions as (w, x, y, z),
rotation matrices row-major, positions as lists of three floats.
"""

import math
import os
import xml.etree.ElementTree as ET


# --------------------------------------------------------------------------
# Small linear algebra on nested lists (row-major)
# --------------------------------------------------------------------------

def mat3_identity():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mat3_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat3_apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat3_transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def mat3_det(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def vec_norm(v):
    return math.sqrt(sum(c * c for c in v))


def vec_normalise(v):
    n = vec_norm(v)
    if n == 0.0:
        raise ValueError("cannot normalise a zero vector")
    return [c / n for c in v]


def vec_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def vec_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def quat_to_mat3(q):
    """MuJoCo quaternion (w, x, y, z) to a rotation matrix."""
    w, x, y, z = vec_normalise(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def mat3_to_quat(m):
    """Rotation matrix to (w, x, y, z). Used for reporting only."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]


def axis_angle_to_mat3(axis, angle):
    x, y, z = vec_normalise(axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def mat4_from(rot, pos):
    return [
        [rot[0][0], rot[0][1], rot[0][2], pos[0]],
        [rot[1][0], rot[1][1], rot[1][2], pos[1]],
        [rot[2][0], rot[2][1], rot[2][2], pos[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat4_identity():
    return mat4_from(mat3_identity(), [0.0, 0.0, 0.0])


def mat4_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat4_rot(m):
    return [row[:3] for row in m[:3]]


def mat4_pos(m):
    return [m[0][3], m[1][3], m[2][3]]


def mat4_apply_point(m, p):
    return [m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3)]


def mat4_apply_vector(m, v):
    return [m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3)]


def mat4_flat(m):
    """Row-major list of 16 floats, the layout Fusion's Matrix3D.setWithArray expects."""
    return [m[i][j] for i in range(4) for j in range(4)]


# --------------------------------------------------------------------------
# Attribute helpers
# --------------------------------------------------------------------------

def _floats(text):
    return [float(t) for t in text.replace(",", " ").split()]


def _angle_scale(compiler):
    return math.pi / 180.0 if compiler.get("angle", "degree") == "degree" else 1.0


def orientation_from_attrs(attrs, compiler):
    """Return a 3x3 rotation from whichever MuJoCo orientation attribute is present."""
    if "quat" in attrs:
        return quat_to_mat3(_floats(attrs["quat"]))
    if "axisangle" in attrs:
        v = _floats(attrs["axisangle"])
        return axis_angle_to_mat3(v[:3], v[3] * _angle_scale(compiler))
    if "euler" in attrs:
        angles = [a * _angle_scale(compiler) for a in _floats(attrs["euler"])]
        seq = compiler.get("eulerseq", "xyz")
        rot = mat3_identity()
        for ch, ang in zip(seq, angles):
            axis = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}[ch.lower()]
            step = axis_angle_to_mat3(axis, ang)
            if ch.islower():
                rot = mat3_mul(rot, step)  # intrinsic: about the moving frame
            else:
                rot = mat3_mul(step, rot)  # extrinsic: about the fixed frame
        return rot
    if "xyaxes" in attrs:
        v = _floats(attrs["xyaxes"])
        x = vec_normalise(v[:3])
        y = v[3:6]
        z = vec_normalise(vec_cross(x, y))
        y = vec_cross(z, x)
        return mat3_transpose([x, y, z])  # axes are the columns
    if "zaxis" in attrs:
        z = vec_normalise(_floats(attrs["zaxis"]))
        ref = [0.0, 0.0, 1.0]
        d = max(-1.0, min(1.0, vec_dot(ref, z)))
        if d > 1 - 1e-12:
            return mat3_identity()
        if d < -1 + 1e-12:
            return axis_angle_to_mat3([1.0, 0.0, 0.0], math.pi)
        return axis_angle_to_mat3(vec_cross(ref, z), math.acos(d))
    return mat3_identity()


def position_from_attrs(attrs, key="pos"):
    return _floats(attrs[key]) if key in attrs else [0.0, 0.0, 0.0]


# --------------------------------------------------------------------------
# Default classes
# --------------------------------------------------------------------------

class DefaultClass:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.attrs = {}  # element tag -> {attr: value}

    def merge(self, elem):
        for child in elem:
            if child.tag == "default":
                continue
            self.attrs.setdefault(child.tag, {}).update(child.attrib)


class Defaults:
    def __init__(self):
        self.classes = {}

    def parse(self, default_elem, parent=None):
        name = default_elem.get("class", "main" if parent is None else None)
        if name is None:
            raise ValueError("nested <default> without a class attribute")
        cls = self.classes.get(name)
        if cls is None:
            cls = DefaultClass(name, parent)
            self.classes[name] = cls
        cls.merge(default_elem)
        for child in default_elem:
            if child.tag == "default":
                self.parse(child, cls)

    def resolve(self, class_name, tag, attr, explicit=None):
        """Explicit attribute first, then the class chain up to 'main'."""
        if explicit is not None and attr in explicit:
            return explicit[attr]
        cls = self.classes.get(class_name) or self.classes.get("main")
        while cls is not None:
            value = cls.attrs.get(tag, {}).get(attr)
            if value is not None:
                return value
            cls = cls.parent
        return None


# --------------------------------------------------------------------------
# Model objects
# --------------------------------------------------------------------------

class Mesh:
    def __init__(self, name, path, scale):
        self.name = name
        self.path = path
        self.scale = scale


class Geom:
    def __init__(self, name, gtype, mesh, pos, rot, group, cls, rgba, material):
        self.name = name
        self.gtype = gtype
        self.mesh = mesh          # mesh asset name or None
        self.pos = pos            # relative to body
        self.rot = rot            # relative to body
        self.group = group
        self.cls = cls
        self.rgba = rgba
        self.material = material
        self.body = None

    @property
    def is_mesh(self):
        return self.mesh is not None and self.gtype in (None, "mesh")


class Joint:
    def __init__(self, name, jtype, pos, axis, rng, cls):
        self.name = name
        self.jtype = jtype        # hinge, slide, ball, free
        self.pos = pos            # relative to body
        self.axis = axis          # relative to body
        self.range = rng          # [min, max] in radians/metres, or None
        self.cls = cls
        self.body = None

    def world_pos(self):
        return mat4_apply_point(self.body.world, self.pos)

    def world_axis(self):
        return vec_normalise(mat4_apply_vector(self.body.world, self.axis))


class Site:
    def __init__(self, name, pos, rot):
        self.name = name
        self.pos = pos
        self.rot = rot
        self.body = None


class Body:
    def __init__(self, name, pos, rot, parent, childclass):
        self.name = name
        self.pos = pos
        self.rot = rot
        self.parent = parent
        self.childclass = childclass
        self.children = []
        self.geoms = []
        self.joints = []
        self.sites = []
        self.world = None         # 4x4, filled by MjcfModel

    @property
    def local(self):
        return mat4_from(self.rot, self.pos)

    @property
    def depth(self):
        d, b = 0, self.parent
        while b is not None:
            d, b = d + 1, b.parent
        return d

    def mesh_geoms(self, groups=None):
        return [g for g in self.geoms if g.is_mesh and (groups is None or g.group in groups)]


# --------------------------------------------------------------------------
# The model
# --------------------------------------------------------------------------

def _expand_includes(elem, base_dir, warnings):
    """Replace <include file="..."/> with the children of the referenced file."""
    i = 0
    while i < len(elem):
        child = elem[i]
        if child.tag == "include":
            path = os.path.join(base_dir, child.get("file", ""))
            if not os.path.isfile(path):
                warnings.append("include not found: %s" % path)
                elem.remove(child)
                continue
            inc_root = ET.parse(path).getroot()
            _expand_includes(inc_root, os.path.dirname(path), warnings)
            elem.remove(child)
            for j, inc_child in enumerate(list(inc_root)):
                elem.insert(i + j, inc_child)
            i += len(inc_root)
            continue
        _expand_includes(child, base_dir, warnings)
        i += 1


class MjcfModel:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        self.dir = os.path.dirname(self.path)
        self.warnings = []
        self.compiler = {}
        self.defaults = Defaults()
        self.meshes = {}
        self.materials = {}      # name -> rgba
        self.bodies = []         # pre-order, world body first
        self.body_by_name = {}
        self.joints = []
        self.sites = []

        root = ET.parse(self.path).getroot()
        self.name = root.get("model", os.path.splitext(os.path.basename(path))[0])
        _expand_includes(root, self.dir, self.warnings)

        for comp in root.findall("compiler"):
            self.compiler.update(comp.attrib)
        for default in root.findall("default"):
            self.defaults.parse(default)
        for asset in root.findall("asset"):
            self._parse_assets(asset)

        self.world = Body("world", [0.0, 0.0, 0.0], mat3_identity(), None, None)
        self.world.world = mat4_identity()
        self.bodies.append(self.world)
        self.body_by_name["world"] = self.world
        for wb in root.findall("worldbody"):
            self._parse_body_children(wb, self.world)

    # -- assets ------------------------------------------------------------

    def _asset_dir(self, key):
        d = self.compiler.get(key) or self.compiler.get("assetdir") or ""
        return d if os.path.isabs(d) else os.path.join(self.dir, d)

    def _parse_assets(self, asset):
        mesh_dir = self._asset_dir("meshdir")
        for m in asset.findall("mesh"):
            cls = m.get("class", "main")
            file_name = self.defaults.resolve(cls, "mesh", "file", m.attrib)
            if file_name is None:
                self.warnings.append("mesh asset without a file (vertex meshes are not supported)")
                continue
            name = m.get("name") or os.path.splitext(os.path.basename(file_name))[0]
            scale_txt = self.defaults.resolve(cls, "mesh", "scale", m.attrib)
            scale = _floats(scale_txt) if scale_txt else [1.0, 1.0, 1.0]
            for unsupported in ("refpos", "refquat"):
                if unsupported in m.attrib:
                    self.warnings.append("mesh %s uses %s, which is ignored" % (name, unsupported))
            path = file_name if os.path.isabs(file_name) else os.path.join(mesh_dir, file_name)
            self.meshes[name] = Mesh(name, os.path.normpath(path), scale)
        for mat in asset.findall("material"):
            name = mat.get("name")
            rgba = self.defaults.resolve(mat.get("class", "main"), "material", "rgba", mat.attrib)
            if name and rgba:
                self.materials[name] = _floats(rgba)

    # -- kinematic tree ----------------------------------------------------

    def _parse_body_children(self, elem, body):
        for child in elem:
            tag = child.tag
            if tag == "body":
                self._parse_body(child, body)
            elif tag == "geom":
                self._parse_geom(child, body)
            elif tag in ("joint", "freejoint"):
                self._parse_joint(child, body, tag)
            elif tag == "site":
                self._parse_site(child, body)
            # inertial, camera, light, frame etc. are ignored

    def _parse_body(self, elem, parent):
        name = elem.get("name") or "body_%d" % len(self.bodies)
        childclass = elem.get("childclass", parent.childclass)
        body = Body(name, position_from_attrs(elem.attrib),
                    orientation_from_attrs(elem.attrib, self.compiler), parent, childclass)
        body.world = mat4_mul(parent.world, body.local)
        parent.children.append(body)
        self.bodies.append(body)
        if name in self.body_by_name:
            self.warnings.append("duplicate body name %s" % name)
        self.body_by_name[name] = body
        self._parse_body_children(elem, body)

    def _resolve(self, elem, body, tag, attr):
        cls = elem.get("class") or body.childclass or "main"
        return self.defaults.resolve(cls, tag, attr, elem.attrib), cls

    def _parse_geom(self, elem, body):
        gtype, cls = self._resolve(elem, body, "geom", "type")
        mesh, _ = self._resolve(elem, body, "geom", "mesh")
        group, _ = self._resolve(elem, body, "geom", "group")
        rgba, _ = self._resolve(elem, body, "geom", "rgba")
        material, _ = self._resolve(elem, body, "geom", "material")
        if mesh is not None and mesh not in self.meshes:
            self.warnings.append("geom in body %s references unknown mesh %s" % (body.name, mesh))
        if mesh is not None and gtype is None:
            gtype = "mesh"
        geom = Geom(elem.get("name"), gtype, mesh, position_from_attrs(elem.attrib),
                    orientation_from_attrs(elem.attrib, self.compiler),
                    int(group) if group is not None else 0, cls,
                    _floats(rgba) if rgba else None, material)
        geom.body = body
        body.geoms.append(geom)

    def _parse_joint(self, elem, body, tag):
        if tag == "freejoint":
            joint = Joint(elem.get("name"), "free", [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], None, "main")
        else:
            jtype, cls = self._resolve(elem, body, "joint", "type")
            axis, _ = self._resolve(elem, body, "joint", "axis")
            rng, _ = self._resolve(elem, body, "joint", "range")
            jtype = jtype or "hinge"
            rng = _floats(rng) if rng else None
            if rng and jtype in ("hinge", "ball"):
                rng = [r * _angle_scale(self.compiler) for r in rng]
            joint = Joint(elem.get("name"), jtype, position_from_attrs(elem.attrib),
                          _floats(axis) if axis else [0.0, 0.0, 1.0], rng, cls)
        joint.body = body
        body.joints.append(joint)
        self.joints.append(joint)

    def _parse_site(self, elem, body):
        site = Site(elem.get("name"), position_from_attrs(elem.attrib),
                    orientation_from_attrs(elem.attrib, self.compiler))
        site.body = body
        body.sites.append(site)
        self.sites.append(site)

    # -- queries -----------------------------------------------------------

    def geom_rgba(self, geom):
        if geom.rgba:
            return geom.rgba
        if geom.material and geom.material in self.materials:
            return self.materials[geom.material]
        return [0.5, 0.5, 0.5, 1.0]

    def geom_label(self, geom, index):
        if geom.name:
            return geom.name
        return "%s_%d" % (geom.mesh or geom.gtype or "geom", index)

    def articulated_bodies(self):
        """Bodies other than the world body, in pre-order."""
        return [b for b in self.bodies if b is not self.world]

    def summary(self):
        n_mesh = sum(len(b.mesh_geoms()) for b in self.bodies)
        return ("model %s: %d bodies, %d joints, %d mesh geoms, %d mesh assets"
                % (self.name, len(self.articulated_bodies()), len(self.joints), n_mesh, len(self.meshes)))
