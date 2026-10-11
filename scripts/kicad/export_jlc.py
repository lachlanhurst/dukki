#!/usr/bin/env python3
"""Write the JLCPCB fabrication and assembly package for the bridge carrier board.

Outputs in hardware/bridge/fab/:
  bridge-gerbers.zip   Gerbers (copper, mask, paste, silkscreen, outline) and Excellon drill files
  bridge-bom.csv       JLC assembly BOM: Comment, Designator, Footprint, LCSC Part #
  bridge-cpl.csv       JLC placement file: Designator, Mid X, Mid Y, Layer, Rotation

Only parts with an LCSC number that are in the BOM and not DNP go to assembly. Hand-fitted parts
(XT30, PH connectors, modules, wire pads, solder jumper) and mounting holes stay out.

Runs on the KiCad VM:  python3 scripts/kicad/export_jlc.py
"""
import csv, subprocess, zipfile
from pathlib import Path
import pcbnew

PCB = Path("hardware/bridge/bridge.kicad_pcb")
OUT = Path("hardware/bridge/fab")
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
with zipfile.ZipFile(OUT / "bridge-gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
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
with open(OUT / "bridge-bom.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
    for (val, fpname, lcsc), refs in sorted(groups.items(), key=lambda kv: refkey(sorted(kv[1], key=refkey)[0])):
        w.writerow([val, ",".join(sorted(refs, key=refkey)), fpname, lcsc])

# CPL: centre of the footprint in board mm, y up from the board's lower-left corner, KiCad rotation
# KiCad 10's Python cannot read the aux origin back as numbers, so take it from the file
import re
m = re.search(r"\(aux_axis_origin ([-\d.]+) ([-\d.]+)\)", PCB.read_text())
if not m:
    raise SystemExit("set the board's drill/place origin first (gen_bridge_pcb.py does)")
x0, y1 = pcbnew.FromMM(float(m.group(1))), pcbnew.FromMM(float(m.group(2)))
with open(OUT / "bridge-cpl.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for fp in sorted(parts, key=lambda p: refkey(p.GetReference())):
        p = fp.GetPosition()
        w.writerow([fp.GetReference(), f"{pcbnew.ToMM(p.x - x0):.3f}mm", f"{pcbnew.ToMM(y1 - p.y):.3f}mm",
                    "Top" if fp.GetLayer() == pcbnew.F_Cu else "Bottom", f"{fp.GetOrientationDegrees() % 360:.1f}"])

print(f"wrote {OUT}: {len(list(GERB.iterdir()))} Gerber/drill files, {len(groups)} BOM lines, {len(parts)} placements")
