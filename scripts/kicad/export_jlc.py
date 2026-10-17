#!/usr/bin/env python3
"""Write the JLCPCB fabrication and assembly package for a board: the bridge carrier (default) or
the head carrier (`head`).

Outputs in hardware/<board>/fab/:
  <board>-gerbers.zip  Gerbers (copper, mask, paste, silkscreen, outline) and Excellon drill files
  <board>-bom.csv      JLC assembly BOM: Comment, Designator, Footprint, LCSC Part #
  <board>-cpl.csv      JLC placement file: Designator, Mid X, Mid Y, Layer, Rotation

Only parts with an LCSC number that are in the BOM and not DNP go to assembly. Hand-fitted parts
(XT30, PH connectors, modules, wire pads, solder jumper) and mounting holes stay out.

Runs on the KiCad VM:  python3 scripts/kicad/export_jlc.py [bridge|head|head_ext]
"""
import csv, subprocess, sys, zipfile
from pathlib import Path
import pcbnew

NAME = sys.argv[1] if len(sys.argv) > 1 else "bridge"
PCB = Path(f"hardware/{NAME}/{NAME}.kicad_pcb")
OUT = Path(f"hardware/{NAME}/fab")
GERB = OUT / "gerbers"
OUT.mkdir(parents=True, exist_ok=True)
for f in GERB.glob("*"):
    f.unlink()
GERB.mkdir(exist_ok=True)

layers = "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
# Gerbers, drill and placement all relative to the board's drill/place origin (its lower-left corner)
subprocess.run(["kicad-cli", "pcb", "export", "gerbers", "--layers", layers, "--subtract-soldermask",
                "--no-x2", "--use-drill-file-origin", "--output", str(GERB) + "/", str(PCB)], check=True, capture_output=True)
subprocess.run(["kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm",
                "--excellon-separate-th", "--drill-origin", "plot", "--output", str(GERB) + "/", str(PCB)],
               check=True, capture_output=True)
with zipfile.ZipFile(OUT / f"{NAME}-gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
    for f in sorted(GERB.iterdir()):
        z.write(f, f.name)

board = pcbnew.LoadBoard(str(PCB))
parts = []
for fp in board.GetFootprints():
    lcsc = fp.GetField("LCSC").GetText() if fp.HasField("LCSC") else ""
    if not lcsc or fp.IsExcludedFromBOM() or fp.IsDNP():
        continue
    parts.append(fp)

# BOM, grouped by value, footprint and LCSC number
groups = {}
for fp in parts:
    key = (fp.GetValue(), fp.GetFPID().GetLibItemName().wx_str(), fp.GetField("LCSC").GetText())
    groups.setdefault(key, []).append(fp.GetReference())
def refkey(r):
    head = r.rstrip("0123456789")
    return head, int(r[len(head):] or 0)
with open(OUT / f"{NAME}-bom.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
    for (val, fpname, lcsc), refs in sorted(groups.items(), key=lambda kv: refkey(sorted(kv[1], key=refkey)[0])):
        w.writerow([val, ",".join(sorted(refs, key=refkey)), fpname, lcsc])

# CPL: centre of the footprint in board mm, y up from the board's lower-left corner, KiCad rotation
# plus a correction where JLC's library footprint (the part's EasyEDA footprint) is drawn turned or
# offset from ours, as their placement preview shows. Rotations found 17/10/2026 by comparing pad
# positions with EasyEDA's footprints (head) and by hand in JLC's preview (bridge, commit a4dfda9):
ROT_FIX = {
    "C597931": 90,     # DF40C CM4 socket: JLC's rows run along x, ours along y (symmetric under 180)
    "C20917": 180,     # AO3400A, SOT-23
    "C8545": 180,      # 2N7002, SOT-23 (bridge)
    "C7420354": 180,   # MMBT3906, SOT-23 (bridge)
    "C45262770": 270,  # LMR51635, SOT-23-6 (head)
    "C21260865": 180,  # LMR51606, SOT-23-6 (bridge)
    "C6186": 180,      # AMS1117-3.3, SOT-223
    "C910544": 270,    # MAX98357A, TQFN-16: square, so only pin 1 shows it
    "C194919": 270,    # BMI088, LGA-16 (head rev B)
}
# JLC's origin offset from ours, in the footprint's own frame (mm, KiCad axes: y down)
POS_FIX = {
    "C130101": (-1.8, 0.0),   # NCE40P70K, TO-252-2 (bridge Q1)
}
# KiCad 10's Python cannot read the aux origin back as numbers, so take it from the file
import math, re
m = re.search(r"\(aux_axis_origin ([-\d.]+) ([-\d.]+)\)", PCB.read_text())
if not m:
    raise SystemExit("set the board's drill/place origin first (gen_bridge_pcb.py and gen_head_pcb.py do)")
x0, y1 = pcbnew.FromMM(float(m.group(1))), pcbnew.FromMM(float(m.group(2)))
with open(OUT / f"{NAME}-cpl.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for fp in sorted(parts, key=lambda p: refkey(p.GetReference())):
        p = fp.GetPosition()
        lcsc = fp.GetField("LCSC").GetText()
        if lcsc in POS_FIX:
            (lx, ly), th = POS_FIX[lcsc], math.radians(fp.GetOrientationDegrees())
            # KiCad turns counter-clockwise as seen, with y down
            p = pcbnew.VECTOR2I(p.x + pcbnew.FromMM(lx * math.cos(th) + ly * math.sin(th)),
                                p.y + pcbnew.FromMM(-lx * math.sin(th) + ly * math.cos(th)))
        w.writerow([fp.GetReference(), f"{pcbnew.ToMM(p.x - x0):.3f}mm", f"{pcbnew.ToMM(y1 - p.y):.3f}mm",
                    "Top" if fp.GetLayer() == pcbnew.F_Cu else "Bottom",
                    f"{(fp.GetOrientationDegrees() + ROT_FIX.get(lcsc, 0)) % 360:.1f}"])

print(f"wrote {OUT}: {len(list(GERB.iterdir()))} Gerber/drill files, {len(groups)} BOM lines, {len(parts)} placements")
