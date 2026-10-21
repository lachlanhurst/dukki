#!/usr/bin/env python3
"""Export an assembled board as STEP, for the repo: hardware/<board>/<board>.step.

    python3 scripts/kicad/export_step.py bridge|head|head_ext

Run on the KiCad machine, from the repo root, after gen_3d_models.py has written the models
in hardware/kicad/3d/. Coordinates are from the board's drill/place origin, its lower-left
corner, as in the fab files; z = 0 on the top face. The board file is not changed: a copy beside
it (so ${KIPRJMOD} still resolves) gets these model changes, is exported with kicad-cli and is
deleted.

- Footprints from DukkiHead.pretty that have no model get the approximate ones from
  gen_3d_models.py (CM4 module, DF40C sockets, camera FPC connector, BMI088 module).
- U4's TQFN-16 3 x 3 has no model in KiCad's library for its 1.23 mm exposed pad; the
  1.6 mm pad one has the same body.
- The WeAct core board's model is derived from WeAct's file, which carries no licence, so the
  committed STEP uses our stand-in instead. For a local export with WeAct's own model, run
  kicad-cli pcb export step on the board directly (docs/bridge-pcb.md 6.4).
"""
import re
import subprocess
import sys
from pathlib import Path

MODELS = "${KIPRJMOD}/../kicad/3d/"
ADD = {
    "DukkiHead:Radxa_CM4_Module": "Radxa_CM4_Module.step",
    "DukkiHead:DF40C_Socket_CM4_1-100": "DF40C_Socket_CM4.step",
    "DukkiHead:DF40C_Socket_CM4_101-200": "DF40C_Socket_CM4.step",
    "DukkiHead:FPC_XUNPU_FPC-1.0HF-15PWAR-H20_1x15-1MP_P1.0mm": "FPC_XUNPU_1x15_P1.0mm_H2.0.step",
    "DukkiHead:BMI088_Module": "BMI088_Module.step",
}
REPLACE = {
    "Package_DFN_QFN.3dshapes/TQFN-16-1EP_3x3mm_P0.5mm_EP1.23x1.23mm.step":
        "Package_DFN_QFN.3dshapes/TQFN-16-1EP_3x3mm_P0.5mm_EP1.6x1.6mm.step",
    "${KIPRJMOD}/../kicad/3d/WeAct_STM32G474_raised.step":
        MODELS + "CoreBoard_STM32G474_standin.step",
}


def block_end(s, start):
    """Index just past the s-expression that opens at s[start]."""
    depth, i, in_str = 0, start, False
    while True:
        c = s[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1


def model(path):
    return (f'\t\t(model "{path}"\n\t\t\t(offset\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n'
            f'\t\t\t(scale\n\t\t\t\t(xyz 1 1 1)\n\t\t\t)\n\t\t\t(rotate\n\t\t\t\t(xyz 0 0 0)\n\t\t\t)\n\t\t)\n\t')


board = sys.argv[1]
src = Path("hardware") / board / f"{board}.kicad_pcb"
s = src.read_text()
for old, new in REPLACE.items():
    s = s.replace(old, new)

out, pos, added = [], 0, 0
for m in re.finditer(r'\(footprint "([^"]+)"', s):
    if m.group(1) not in ADD:
        continue
    end = block_end(s, m.start())
    if "(model " in s[m.start():end]:
        continue
    out.append(s[pos:end - 1])
    out.append(model(MODELS + ADD[m.group(1)]))
    pos, added = end - 1, added + 1
out.append(s[pos:])

tmp = src.with_name(f"{board}-step-export.kicad_pcb")
tmp.write_text("".join(out))
dst = src.with_suffix(".step")
try:
    r = subprocess.run(["kicad-cli", "pcb", "export", "step", "--subst-models", "--drill-origin",
                        "--force", "-o", str(dst), str(tmp)], capture_output=True, text=True)
finally:
    tmp.unlink()
for line in (r.stdout + r.stderr).splitlines():
    if "No enum choices" not in line:
        print(line)
print(f"{added} models added; wrote {dst}")
sys.exit(r.returncode)
