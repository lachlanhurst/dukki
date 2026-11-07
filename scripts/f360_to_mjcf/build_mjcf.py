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

OLD_BATTERY_KG = 0.100   # NP-F550 the shipped robot carries (docs/hardware.md section 10)
NEW_BATTERY_KG = 0.132   # 6S pack, weighed 30/09/2026

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


def write_meshes():
    """Decimate the Fusion exports (mm, component frame) and write them to the assets folder in metres."""
    stats = {}
    for f in sorted(os.listdir(S + 'raw')):
        name = f[:-4]
        mesh = trimesh.load(S + 'raw/' + f)
        mesh.merge_vertices()
        n0 = len(mesh.faces)
        if n0 > MAX_FACES:
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


def shift(I_c, mass, c):
    """Inertia about the origin from inertia about the CoM c."""
    return I_c + mass * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def new_trunk_inertial(inertial, old_batt_T, old_batt_half, new_batt_T, new_batt_half):
    """Swap the battery inside the trunk inertial. Poses in the trunk frame, metres."""
    m0 = float(inertial.get('mass'))
    c0 = np.array([float(x) for x in inertial.get('pos').split()])
    xx, yy, zz, xy, xz, yz = [float(x) for x in inertial.get('fullinertia').split()]
    I0 = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    if inertial.get('quat'):
        raise ValueError('rotated trunk inertial not handled')

    def part(mass, T, half):
        R, c = T[:3, :3], T[:3, 3]
        return mass, c, shift(R @ box_inertia(mass, half) @ R.T, mass, c)

    mo, co, Io = part(OLD_BATTERY_KG, old_batt_T, old_batt_half)
    mn, cn, In = part(NEW_BATTERY_KG, new_batt_T, new_batt_half)
    m1 = m0 - mo + mn
    c1 = (m0 * c0 - mo * co + mn * cn) / m1
    I_origin = shift(I0, m0, c0) - Io + In
    I1 = I_origin - m1 * (np.dot(c1, c1) * np.eye(3) - np.outer(c1, c1))
    assert np.all(np.linalg.eigvalsh(I1) > 0)
    inertial.set('pos', fmt(c1))
    inertial.set('mass', '%.6g' % m1)
    inertial.set('fullinertia', fmt([I1[0, 0], I1[1, 1], I1[2, 2], I1[0, 1], I1[0, 2], I1[1, 2]]))
    return m0, m1, c0, c1


def old_battery_in_trunk(body):
    """NP-F970-shaped battery geom in the old trunk: pose (trunk frame, m) and half sizes (m) of its oriented box."""
    for g in body.findall('geom'):
        if g.get('mesh') == 'np_f970' and g.get('class') == 'visual':
            T = np.eye(4)
            T[:3, :3] = trimesh.transformations.quaternion_matrix([float(x) for x in g.get('quat').split()])[:3, :3]
            T[:3, 3] = [float(x) for x in g.get('pos').split()]
            mesh = trimesh.load(ASSETS_OLD + 'np_f970.stl')
            to, ext = trimesh.bounds.oriented_bounds(mesh)
            return T @ np.linalg.inv(to), ext / 2
    raise KeyError('np_f970')


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
        mine = [u for u in U if LINK_BODY[u[0]] == bname]
        if not mine:
            continue
        # collision roles this variant gave the old meshes of this body
        roles = {}
        for g in body.findall('geom'):
            mesh = g.get('mesh')
            if mesh in OLD2NEW and g.get('class') in ('collision', 'self_collision_only'):
                for n in OLD2NEW[mesh]:
                    roles[n] = (g.get('class'), g.get('name'))
        if bname == 'trunk_base':
            old_bT, old_bh = old_battery_in_trunk(body)
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
        for link, comp, name, T in mine:
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

        if bname == 'trunk_base':
            bT, half = battery_box()
            nbT = Xinv @ bT
            nbT[:3, 3] /= 1000.0
            res = new_trunk_inertial(body.find('inertial'), old_bT, old_bh, nbT, half / 1000.0)
            log.append('trunk mass %.4f -> %.4f kg, CoM %s -> %s' % (res[0], res[1], fmt(res[2]), fmt(res[3])))
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


PROVENANCE = ('<!-- Geometry, battery and IMU site replaced from the Dukki Fusion 360 model (claude_test, Microduck project);'
              ' joints, bodies and the other inertials are the Onshape export\'s. -->')


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
    ASSETS_OLD = S + 'assets_old/'
    if not os.path.isdir(ASSETS_OLD):
        import shutil
        shutil.copytree(ASSETS, ASSETS_OLD)
    load_old_rgba()
    for k, v in write_meshes().items():
        print('mesh %-32s %6d -> %5d faces, %7d bytes' % (k, *v))
    for v in VARIANTS:
        print('==', v)
        for line in rebuild(v, set()):
            print('  ', line)
