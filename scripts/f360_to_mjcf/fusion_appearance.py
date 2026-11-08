"""Fusion 360 script: dump the appearance of every part in claude_test.

Writes into OUT_DIR:
- appearance.json: for each component (the first occurrence of each), every body's effective
  appearance (an appearance set on an occurrence overrides those below it, as Fusion shows it),
  its colour properties and surface roughness, and its area;
- split/<component>/<i>.stl, for the components in SPLIT: one low-resolution STL per body, in
  the frame of the component that owns the body (mm), with that occurrence's world transform in
  appearance.json, so build_mjcf.py can colour the part by body.

Run it with the claude_test design active, through the Fusion MCP server or as a script. It
switches hidden parts on so they export, then raises to roll that back, so the design is left
unchanged; the final "appearance done" error is expected.
"""
import json
import os

import adsk.core
import adsk.fusion

OUT_DIR = os.path.expanduser('~/not_work/microduck/repo/microduck-unitree/scripts/f360_to_mjcf/work')
# parts whose appearance is split fairly evenly between several, coloured body by body
SPLIT = ('bridge v1', 'head_compute v7', 'camera IMX219 v4')

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
dukki = root.occurrences.itemByName('dukki:1')


def props(a):
    out = {}
    for p in a.appearanceProperties:
        if p.objectType == adsk.core.ColorProperty.classType():
            v = adsk.core.ColorProperty.cast(p).value
            if v:
                out[p.id] = [v.red, v.green, v.blue, v.opacity]
        elif p.id == 'surface_roughness':
            out[p.id] = p.value
    return out


def walk(o, override, top, acc):
    override = o.appearance or override
    for b in o.bRepBodies:
        a = override or b.appearance
        acc.append({'body': b, 'app': a.name, 'props': props(a), 'area': b.area,
                    'T': [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] if o is top else list(o.transform2.asArray())})
    for c in o.childOccurrences:
        walk(c, override, top, acc)
    return acc


def show(o):
    o.isLightBulbOn = True
    for b in o.bRepBodies:
        b.isLightBulbOn = True
    for c in o.childOccurrences:
        show(c)


data, seen = {}, set()
for comp in (root, dukki.component):
    for g in comp.rigidGroups:
        for o in g.occurrences:
            levels = o.fullPathName.split('+')
            if o.fullPathName.startswith('joint_frames') or not (len(levels) == 1 or (len(levels) == 2 and levels[0] == 'dukki:1')):
                continue
            name = o.component.name
            if name in seen:
                continue
            seen.add(name)
            bodies = walk(o, None, o, [])
            entry = {'T_top': list(o.transform2.asArray()), 'bodies': []}
            if name in SPLIT:
                show(o)
                folder = os.path.join(OUT_DIR, 'split', name)
                os.makedirs(folder, exist_ok=True)
            for i, b in enumerate(bodies):
                rec = {k: b[k] for k in ('app', 'props', 'area', 'T')}
                if name in SPLIT:
                    f = os.path.join(folder, '%d.stl' % i)
                    opt = design.exportManager.createSTLExportOptions(b['body'], f)
                    opt.isBinaryFormat = True
                    opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    opt.sendToPrintUtility = False
                    design.exportManager.execute(opt)
                    rec['stl'] = '%d.stl' % i if os.path.exists(f) else None
                entry['bodies'].append(rec)
            data[name] = entry

with open(os.path.join(OUT_DIR, 'appearance.json'), 'w') as fh:
    json.dump(data, fh, indent=1)
raise Exception('appearance done, visibility rolled back')


def run(context):
    pass
