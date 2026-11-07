"""Fusion 360 script: dump what the MJCF conversion needs from claude_test.

Writes into OUT_DIR:
- fusion_data.json: every rigid group's top-level parts with their world transforms, the
  joint_frames axis lines, the LSM6DSV16X occurrence and the battery's box;
- raw/<mesh>.stl: one low-resolution STL per unique part, in the part's own frame, in mm.

Run it with the claude_test design active, through the Fusion MCP server or as a script. It
switches hidden parts on so they export, then raises to roll that back, so the design is left
unchanged; the final "export done" error is expected.
"""
import json
import os

import adsk.core
import adsk.fusion

OUT_DIR = os.path.expanduser('~/not_work/microduck/repo/microduck-unitree/scripts/f360_to_mjcf/work')
NAMES_JSON = os.path.expanduser('~/not_work/microduck/repo/microduck-unitree/scripts/f360_to_mjcf/names.json')
SKIP = ('trunk_shell_left',)   # an empty leftover body in claude_test

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
dukki = root.occurrences.itemByName('dukki:1')
names = json.load(open(NAMES_JSON))


def count_bodies(comp):
    return comp.bRepBodies.count + sum(count_bodies(o.component) for o in comp.occurrences)


def find(occs, comp_name):
    for o in occs:
        if o.component.name == comp_name:
            return o
        hit = find(o.childOccurrences, comp_name)
        if hit:
            return hit
    return None


def by_path(path):
    occs, o = root.occurrences, None
    for n in path.split('+'):
        o = occs.itemByName(n)
        occs = o.childOccurrences
    return o


def show(o):
    o.isLightBulbOn = True
    for c in o.childOccurrences:
        show(c)


def bbox(body):
    b = body.boundingBox
    return [b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z]


data = {'links': {}, 'axes': {}}
for comp in (root, dukki.component):
    for g in comp.rigidGroups:
        parts = []
        for o in g.occurrences:
            path = o.fullPathName
            levels = path.split('+')
            if path.startswith('joint_frames'):
                continue
            if len(levels) == 1 or (len(levels) == 2 and levels[0] == 'dukki:1'):
                parts.append({'path': path, 'comp': o.component.name, 'T': list(o.transform2.asArray()),
                              'visible': o.isLightBulbOn and o.isVisible, 'nb': count_bodies(o.component)})
        data['links'][g.name] = parts

frames = root.occurrences.itemByName('joint_frames:1')
for o in frames.childOccurrences:
    for sk in o.component.sketches:
        if sk.name.startswith('axis_'):
            ln = sk.sketchCurves.sketchLines.item(0)
            a, b = ln.startSketchPoint.worldGeometry, ln.endSketchPoint.worldGeometry
            data['axes'][sk.name[5:]] = [a.x, a.y, a.z, b.x, b.y, b.z]

imu = find(root.occurrences, 'lsm6dsv16x')
data['imu'] = {'path': imu.fullPathName, 'T': list(imu.transform2.asArray()),
               'bbox': bbox(imu.component.bRepBodies.item(0))}
battery = dukki.childOccurrences.itemByName('battery 6s lipo:1')
data['battery'] = {'T': list(battery.transform2.asArray()), 'bbox': bbox(battery.component.bRepBodies.item(0)),
                   'vol': battery.component.bRepBodies.item(0).volume}

os.makedirs(os.path.join(OUT_DIR, 'raw'), exist_ok=True)
with open(os.path.join(OUT_DIR, 'fusion_data.json'), 'w') as fh:
    json.dump(data, fh, indent=1)

first = {}
for parts in data['links'].values():
    for p in parts:
        if p['comp'] not in SKIP:
            first.setdefault(p['comp'], p['path'])
missing = sorted(c for c in first if c not in names)
if missing:
    raise Exception('add these components to names.json: %s' % missing)

em = design.exportManager
report = []
for comp, path in first.items():
    o = by_path(path)
    show(o)
    out = os.path.join(OUT_DIR, 'raw', names[comp] + '.stl')
    opt = em.createSTLExportOptions(o, out)
    opt.isBinaryFormat = True
    opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
    opt.sendToPrintUtility = False
    em.execute(opt)
    report.append('%s %d' % (names[comp], os.path.getsize(out) if os.path.exists(out) else -1))
print('\n'.join(report))
raise Exception('export done, visibility rolled back')


def run(context):
    pass
