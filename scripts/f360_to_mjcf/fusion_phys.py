"""Fusion 360 script: dump every part's physical properties from claude_test.

Writes OUT_DIR/fusion_phys.json: for each top-level part of each rigid group (the same parts
fusion_export.py lists), its mass (kg), centre of mass (claude_test world, cm) and inertia
tensor about the world origin (kg.cm^2, tensor form, as getXYZMomentsOfInertia returns it).
The masses come from the bodies' physical materials, so set those first (PA12 SLS for the
printed parts). Read-only.
"""
import json
import os

import adsk.core
import adsk.fusion

OUT_DIR = os.path.expanduser('~/not_work/microduck/repo/microduck-unitree/scripts/f360_to_mjcf/work')

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
dukki = root.occurrences.itemByName('dukki:1')
ACC = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy

out = {}
for comp in (root, dukki.component):
    for g in comp.rigidGroups:
        for o in g.occurrences:
            path = o.fullPathName
            levels = path.split('+')
            if path.startswith('joint_frames'):
                continue
            if not (len(levels) == 1 or (len(levels) == 2 and levels[0] == 'dukki:1')):
                continue
            pp = o.getPhysicalProperties(ACC)
            c = pp.centerOfMass
            _, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()
            out[path] = {'comp': o.component.name, 'mass': pp.mass, 'volume': pp.volume, 'com': [c.x, c.y, c.z],
                         'I': [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]}

with open(os.path.join(OUT_DIR, 'fusion_phys.json'), 'w') as fh:
    json.dump(out, fh, indent=1)
print('%d parts written' % len(out))


def run(context):
    pass
