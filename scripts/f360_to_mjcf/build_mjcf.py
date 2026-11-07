"""Rewrite the microduck MJCF variants with geometry from the Dukki Fusion model (claude_test)."""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *  # noqa: E402,F403

ASSETS = RL + 'assets/'
VARIANTS = ['robot_walk', 'robot_walk_backlash', 'robot_groundcontact', 'robot_groundcontact_backlash',
            'robot_groundcontact_rollers', 'robot_groundcontact_rollers_backlash', 'robot_allcollisions']
ROLLER_MESHES = {'rim', 'roller_blade', 'tire', 'ankle_l_v1', 'ankle_r_v1'}
MAX_FACES = 2500
# MuJoCo collides against a mesh's convex hull, and decimation leaves the soles' flat undersides
# bumpy, so the soles keep the full Fusion export
FULL_RES = {'sole_left', 'sole_right'}

# every part's physical properties from fusion_phys.py, and the masses that override them
PHYS = json.load(open(S + 'fusion_phys.json'))
MASSES = {k: v for k, v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'masses.json'))).items()
          if not k.startswith('_')}
PENDING = set()

# old Onshape mesh -> new Fusion meshes that take over its role (visual and collision)
OLD2NEW = {
    'right_shell': ['right_shell'], 'left_shell': ['left_shell'], 'trunk_base': ['trunk_base'], 'xl330': ['j288'],
    'seeed_bearing__configuration__22x16x4': ['bearing_61702'], 'seeed_bearing__configuration_default': ['bearing_6700'],
    'np_f970': ['battery'], 'power_support': ['bridge_mount'], 'banana_pcb_locker': ['bridge'],
    'yaw2roll': ['yaw2roll'], 'bearing_roll': ['bearing_roll'], 'hip_l': ['hip_l'],
    'upper_leg_left': ['upper_leg_left'], 'upper_leg_right': ['upper_leg_right'],
    'upper_leg_rigidity_plate': ['upper_leg_rigidity_plate', 'upper_leg_rigidity_plate_right'],
    'leg': ['leg'], 'sole_left': ['sole_left'], 'sole_right': ['sole_right'],
    'foot_left': ['foot_left'], 'ankle_left': ['foot_left'], 'foot_right': ['foot_right'], 'ankle_right': ['foot_right'],
    'neck': ['neck'], 'neck_pitch': ['neck_pitch'], 'yaw_roll_motion': ['yaw_roll_motion'],
    'lens': ['camera'], 'm12_lens_holder': ['camera'], 'face_part': ['face_part'], 'jaw_soft': ['jaw_soft'],
    'motor_support': ['motor_support'], 'top_head_shell': ['top_head_shell'],
    'elec_rpi_robot_hat_pcb': ['head_compute'], 'pcb__raspberry_pi_zero_2_w': ['head_compute'],
    'soft_mouth_top': ['soft_mouth_top'], 'noenoeil': ['noenoeil'], 'jaw': ['jaw'],
    'bottom_head_shell': ['bottom_head_shell'], 'speaker': ['speaker'],
}
# colours: reuse the old part's material where one exists
COLOUR_FROM = {
    'j288': 'xl330', 'bearing_61702': 'seeed_bearing__configuration__22x16x4',
    'bearing_6700': 'seeed_bearing__configuration_default', 'battery': 'np_f970', 'bridge_mount': 'power_support',
    'bridge': 'elec_rpi_robot_hat_pcb', 'head_compute': 'elec_rpi_robot_hat_pcb', 'camera': 'lens',
    'foot_left': 'foot_left', 'foot_right': 'foot_right', 'upper_leg_rigidity_plate_right': 'upper_leg_rigidity_plate',
}
NEW_COLOURS = {
    'oled': '0.1 0.1 0.1 1', 'switch': '0.15 0.15 0.15 1', 'tof': '0.1 0.1 0.1 1', 'mic': '0.313725 0.486275 0.411765 1',
    'led': '0.313725 0.486275 0.411765 1', 'head_compute_clamp': '0.129412 0.152941 0.129412 1',
    'head_compute_front_clamp': '0.129412 0.152941 0.129412 1', 'fan': '0.15 0.15 0.15 1',
}


def fmt(v):
    return ' '.join('%.6g' % (0.0 if abs(x) < 5e-10 else x) for x in v)


def pose_attrs(T):
    """4x4 (mm) -> MJCF pos (m) and quat (w x y z) strings."""
    q = trimesh.transformations.quaternion_from_matrix(T)
    if q[0] < 0:
        q = -q
    return fmt(T[:3, 3] / 1000.0), fmt(q)


def j288_mesh():
    """The J288 as primitives, in its component frame (mm): the housing as a box, the driven
    horn (on the flat end, -x) as a cylinder. The non-driven horn sits in the inset end inside
    the box's envelope and is left out, as are the mounting holes."""
    body = trimesh.creation.box(bounds=[[-3.5, -17.0, -10.0], [19.5, 17.0, 10.0]])
    horn = trimesh.creation.cylinder(radius=7.0, segment=[[-6.5, 7.5, 0.0], [-3.5, 7.5, 0.0]], sections=24)
    return trimesh.util.concatenate([body, horn])


def write_meshes():
    """Decimate the Fusion exports (mm, component frame) and write them to the assets folder in metres."""
    stats = {}
    excluded = {NAMES[c] for c, v in MASSES.items() if v.get('exclude') and c in NAMES}
    for f in sorted(os.listdir(S + 'raw')):
        name = f[:-4]
        if name in excluded:
            continue
        mesh = trimesh.load(S + 'raw/' + f)
        mesh.merge_vertices()
        n0 = len(mesh.faces)
        if name == 'j288':
            mesh = j288_mesh()
        elif n0 > MAX_FACES and name not in FULL_RES:
            mesh = mesh.simplify_quadric_decimation(face_count=MAX_FACES)
        mesh.apply_scale(0.001)
        mesh.export(ASSETS + name + '.stl')
        stats[name] = (n0, len(mesh.faces), os.path.getsize(ASSETS + name + '.stl'))
    return stats


def battery_box():
    """Battery box pose (4x4, mm, MJCF world at zero pose) and half sizes (mm)."""
    lo = np.array(F['battery']['bbox'][:3]) * 10
    hi = np.array(F['battery']['bbox'][3:]) * 10
    T = C['battery'] @ fusion_T(F['battery']['T'])
    Tc = np.eye(4)
    Tc[:3, 3] = (lo + hi) / 2
    return T @ Tc, (hi - lo) / 2


def box_inertia(mass, half):
    a, b, c = 2 * half
    return mass / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


def part_inertial(comp, path, link):
    """One part's mass (kg), centre of mass (MJCF world at the zero pose, mm) and inertia tensor
    about it (MJCF world axes, kg.mm^2): Fusion's physical properties, or the masses.json override."""
    p = PHYS[path]
    over = MASSES.get(comp, {})
    m = p['mass']
    c = np.array(p['com']) * 10.0
    I_c = (np.array(p['I']) - m * (np.dot(p['com'], p['com']) * np.eye(3) - np.outer(p['com'], p['com']))) * 100.0
    if over.get('box'):
        T, half = battery_box()
        m = over['mass']
        R = T[:3, :3]
        return m, T[:3, 3], R @ box_inertia(m, half) @ R.T
    if 'mass' in over:
        if over['mass'] is None:
            if not os.environ.get('F360_MJCF_PENDING_OK'):
                raise ValueError('%s has no weighed mass yet (masses.json)' % comp)
            PENDING.add(comp)
        else:
            I_c = I_c * over['mass'] / m
            m = over['mass']
            if 'com' in over:
                c = fusion_T(next(u['T'] for us in F['links'].values() for u in us if u['path'] == path))[:3, :] @ np.r_[over['com'], 1.0]
    Ck = C[link]
    return m, Ck[:3, :3] @ c + Ck[:3, 3], Ck[:3, :3] @ I_c @ Ck[:3, :3].T


def set_inertial(inertial, parts, Xinv):
    """Replace a body's inertial with the sum of its parts, in the body frame (m, kg.m^2)."""
    m0 = float(inertial.get('mass'))
    items = []
    for link, comp, name, T, path in parts:
        m, c, I_c = part_inertial(comp, path, link)
        R = Xinv[:3, :3]
        items.append((m, R @ c + Xinv[:3, 3], R @ I_c @ R.T))
    M = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / M
    I = np.zeros((3, 3))
    for m, c, I_c in items:
        d = c - com
        I += I_c + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I *= 1e-6
    assert np.all(np.linalg.eigvalsh(I) > 0)
    for a in ('quat', 'diaginertia'):
        inertial.attrib.pop(a, None)
    inertial.set('pos', fmt(com / 1000.0))
    inertial.set('mass', '%.6g' % M)
    inertial.set('fullinertia', fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
    return m0, M


def rebuild(variant, materials):
    path = RL + variant + '.xml'
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(path, parser)
    root = tree.getroot()
    _, _, X = body_poses(path)
    all_collide = variant == 'robot_allcollisions'
    U = units()
    log = []

    for body in root.iter('body'):
        bname = body.get('name')
        mine = [u for u in U if LINK_BODY[u[0]] == bname and not MASSES.get(u[1], {}).get('exclude')]
        if not mine:
            continue
        # collision roles this variant gave the old meshes of this body
        roles = {}
        for g in body.findall('geom'):
            mesh = g.get('mesh')
            if mesh in OLD2NEW and g.get('class') in ('collision', 'self_collision_only'):
                for n in OLD2NEW[mesh]:
                    roles[n] = (g.get('class'), g.get('name'))
        # drop the old Onshape geoms and their "Part" comments
        children = list(body)
        idx = None
        for i, ch in enumerate(children):
            is_old_geom = ch.tag == 'geom' and ch.get('mesh') and ch.get('mesh') not in ROLLER_MESHES
            is_part_comment = ch.tag is ET.Comment and ch.text.strip().startswith('Part ')
            if is_part_comment and i + 1 < len(children) and children[i + 1].get('mesh') in ROLLER_MESHES:
                continue
            if is_old_geom or is_part_comment:
                idx = i if idx is None else idx
                body.remove(ch)
        if idx is None:
            idx = max(i for i, ch in enumerate(list(body)) if ch.tag in ('inertial', 'joint', 'freejoint')) + 1
        Xinv = np.linalg.inv(X[bname])
        new = []
        counts = {}
        for link, comp, name, T, _ in mine:
            counts[name] = counts.get(name, 0) + 1
            label = name if counts[name] == 1 else '%s_%d' % (name, counts[name])
            pos, quat = pose_attrs(Xinv @ T)
            new.append(ET.Comment(' Part %s ' % label))
            attrs = {'type': 'mesh', 'class': 'visual', 'pos': pos, 'quat': quat, 'mesh': name, 'material': name + '_material'}
            new.append(ET.Element('geom', attrs))
            materials.add(name)
            role = roles.get(name)
            if role is None and all_collide:
                role = ('self_collision_only', None) if name == 'bridge_mount' else ('collision', None)
            if role is None:
                continue
            cls, gname = role
            if name == 'battery':
                bT, half = battery_box()
                bpos, bquat = pose_attrs(Xinv @ bT)
                c = {'type': 'box', 'class': cls, 'size': fmt(half / 1000.0), 'pos': bpos, 'quat': bquat, 'material': name + '_material'}
            else:
                c = dict(attrs, **{'class': cls})
            if gname:
                c = dict({'type': c['type'], 'name': gname}, **{k: v for k, v in c.items() if k != 'type'})
            new.append(ET.Element('geom', c))
            log.append('%s/%s %s%s' % (bname, label, cls, ' ' + gname if gname else ''))
        for k, el in enumerate(new):
            body.insert(idx + k, el)

        m0, m1 = set_inertial(body.find('inertial'), mine, Xinv)
        log.append('%s mass %.4f -> %.4f kg' % (bname, m0, m1))

        if bname == 'trunk_base':
            # IMU: the chip centre, trunk-aligned axes for the existing consumers, plus the chip's own frame
            lo = np.array(F['imu']['bbox'][:3]) * 10
            hi = np.array(F['imu']['bbox'][3:]) * 10
            Ti = C['trunk'] @ fusion_T(F['imu']['T'])
            Tc = np.eye(4)
            Tc[:3, 3] = (lo + hi) / 2
            Ti = Xinv @ Ti @ Tc
            ipos, iquat = pose_attrs(Ti)
            children = list(body)
            for i, ch in enumerate(children):
                if ch.tag == 'site' and ch.get('name') == 'imu':
                    ch.set('pos', ipos)
                    ch.set('quat', '1 0 0 0')
                    body.insert(i + 1, ET.Comment(' Frame imu_chip: the LSM6DSV16X axes (x left, y down, z rear) '))
                    body.insert(i + 2, ET.Element('site', {'group': '3', 'name': 'imu_chip', 'pos': ipos, 'quat': iquat}))
                    log.append('imu site %s, chip quat %s' % (ipos, iquat))
                    break

    # assets: drop meshes/materials nothing references any more, add the new ones
    asset = root.find('asset')
    used_meshes = {g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    used_mats = {g.get('material') for g in root.iter('geom') if g.get('material')}
    have = set()
    for el in list(asset):
        if el.tag == 'mesh':
            n = el.get('file')[:-4]
            if n not in used_meshes:
                asset.remove(el)
            else:
                have.add(n)
        elif el.tag == 'material' and el.get('name') not in used_mats:
            asset.remove(el)
    pos = max(i for i, el in enumerate(list(asset)) if el.tag == 'mesh') + 1 if have else 0
    for n in sorted(used_meshes - have):
        asset.insert(pos, ET.Element('mesh', {'file': n + '.stl'}))
        pos += 1
    have_mats = {el.get('name') for el in asset if el.tag == 'material'}
    for n in sorted(materials):
        mn = n + '_material'
        if mn in used_mats and mn not in have_mats:
            asset.append(ET.Element('material', {'name': mn, 'rgba': material_rgba(n)}))
    ET.indent(tree, '  ')
    original = open(path).read()
    head = original[original.index('?>') + 2:original.index('<mujoco')].strip()
    with open(path, 'w') as fh:
        fh.write('<?xml version="1.0" ?>\n')
        if head:
            fh.write(head + '\n')
        fh.write(PROVENANCE + '\n')
        fh.write(ET.tostring(root, encoding='unicode').replace(' />', '/>'))
        fh.write('\n')
    return log


PROVENANCE = ('<!-- Geometry, inertials and IMU site from the Dukki Fusion 360 model (claude_test, Microduck project):'
              ' printed parts from their PA12 SLS / TPU materials, bought parts weighed (f360_to_mjcf masses.json);'
              ' joints and bodies are the Onshape export\'s. -->')


OLD_RGBA = {}


def load_old_rgba():
    for v in VARIANTS:
        for el in ET.parse(RL + v + '.xml').getroot().iter('material'):
            OLD_RGBA[el.get('name')[:-len('_material')]] = el.get('rgba')


def material_rgba(n):
    if n in NEW_COLOURS:
        return NEW_COLOURS[n]
    return OLD_RGBA[COLOUR_FROM.get(n, n)]


if __name__ == '__main__':
    load_old_rgba()
    for k, v in write_meshes().items():
        print('mesh %-32s %6d -> %5d faces, %7d bytes' % (k, *v))
    for v in VARIANTS:
        print('==', v)
        for line in rebuild(v, set()):
            print('  ', line)
    if PENDING:
        print('NOT WEIGHED YET, Fusion mass kept:', ', '.join(sorted(PENDING)))
