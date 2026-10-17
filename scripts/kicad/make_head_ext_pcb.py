#!/usr/bin/env python3
"""Make the external IMU head board (hardware/head_ext/) from the routed head board, keeping its
placement and routing (docs/head-pcb.md section 9.6).

The module sits where rev A had it, 0.4 mm lower (pin row at y 41.91) so its courtyard clears JP3's.
J12 and the I2C pins (J3, 2.0 mm pitch to fit above the camera socket) share the right edge, beside
the module, where the old I2C header was; the rows of bottom tracks above the module stay as routed.

The board is a copy of hardware/head/head.kicad_pcb; sync_head_pcb.py --board head_ext takes out
the BMI088 chip and what came with it and puts the module and the moved headers; this script swaps
the silkscreen that named the old parts. One stage per process (pcbnew crashes when a board is
edited further after removals). Runs on the KiCad VM from the repo root:
    python3 scripts/kicad/gen_head_sch.py --variant head_ext
    cp hardware/head/head.kicad_pcb hardware/head_ext/head_ext.kicad_pcb
    python3 scripts/kicad/make_head_ext_pcb.py project       # the head's rules and severities
    kicad-cli sch export netlist -o /tmp/head_ext.net hardware/head_ext/head_ext.kicad_sch
    python3 scripts/kicad/sync_head_pcb.py /tmp/head_ext.net --board head_ext --prune --replace J12
    python3 scripts/kicad/make_head_ext_pcb.py unsilk
    python3 scripts/kicad/sync_head_pcb.py /tmp/head_ext.net --board head_ext \\
        --place U5 49.67 41.91 0 --place J12 71.0 28.8 0 --place J3 71.0 39.75 0 --place J11 64.91 36.0 0 back
    python3 scripts/kicad/make_head_ext_pcb.py silk
    python3 scripts/kicad/rip_under_pads.py hardware/head_ext/head_ext.kicad_pcb U5 J3 J11 J12
    python3 scripts/kicad/tidy_head_pcb.py undangle head_ext all     # until it removes nothing
then route the broken nets (route_one_net.py with PCB=hardware/head_ext/head_ext.kicad_pcb) and tidy
(tidy_head_pcb.py <stage> head_ext).
"""
import json, sys
from pathlib import Path

if sys.argv[1] == "project":
    d = json.loads(Path("hardware/head/head.kicad_pro").read_text())
    d["meta"]["filename"] = "head_ext.kicad_pro"
    d["schematic"]["top_level_sheets"] = [{**t, "filename": "head_ext.kicad_sch", "name": "head_ext"}
                                          for t in d["schematic"]["top_level_sheets"]]
    Path("hardware/head_ext/head_ext.kicad_pro").write_text(json.dumps(d, indent=2) + "\n")
    raise SystemExit("wrote hardware/head_ext/head_ext.kicad_pro")

import pcbnew
PCB = Path("hardware/head_ext/head_ext.kicad_pcb")
O = (100.0, 50.0)
mm, tomm = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(str(PCB))
def pt(x, y): return pcbnew.VECTOR2I(mm(O[0] + x), mm(O[1] + y))


def silk(text, x, y, size=0.8, rot=0, back=False):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(text); t.SetPosition(pt(x, y)); t.SetLayer(pcbnew.B_SilkS if back else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15))
    t.SetTextAngleDegrees(rot); t.SetMirrored(back)
    b.Add(t)


def imu_labels():
    """The module's pin names, both sides, the INT row's turned 90 degrees (four letters do not fit
    a 2.54 mm pitch flat). Front: inside the outline, the 7-pin row's above its pads and the INT
    row's below. Back: outside the pin rows (between them are J11's pads and names)."""
    u5 = b.FindFootprintByReference("U5")
    names = {"1": "VCC", "2": "GND", "3": "SCL", "4": "SDA", "5": "SDO", "6": "CSA", "7": "CSG",
             "8": "INT4", "9": "INT3", "10": "INT2", "11": "INT1"}
    for back in (False, True):
        for pad in u5.Pads():
            x, y = tomm(pad.GetPosition().x) - O[0], tomm(pad.GetPosition().y) - O[1]
            if int(pad.GetNumber()) <= 7:
                silk(names[pad.GetNumber()], x, y + (1.9 if back else -1.9), back=back)
            else:
                silk(names[pad.GetNumber()], x, y + (-2.4 if back else 2.4), rot=90, back=back)


if sys.argv[1] == "imusilk":     # added to a board made before imu_labels() was part of the silk stage
    imu_labels()
elif sys.argv[1] == "unsilk":
    # the old J11/J12 names and pad labels (front and back), the IMU cut note, the J3 name and the rev line
    gone = []
    for d in b.GetDrawings():
        if d.Type() != pcbnew.PCB_TEXT_T:
            continue
        x, y = tomm(d.GetPosition().x) - O[0], tomm(d.GetPosition().y) - O[1]
        if (46 < x < 67 and 33 < y < 45) or d.GetText() in ("CUT: EXT IMU", "I2C/IMU", "rev B 2026", "CUT: AMP") \
                or (d.GetText() == "1" and 68 < x < 71 and 29 < y < 31):
            gone.append(d)
    for d in gone:
        print("removed", repr(d.GetText()), "back" if d.GetLayer() == pcbnew.B_SilkS else "front",
              round(tomm(d.GetPosition().x) - O[0], 2), round(tomm(d.GetPosition().y) - O[1], 2))
        b.Remove(d)
    # the BMI088 chip's ground fan-out, left behind with its pads gone (ground is not pruned with
    # the nets, and the undangle stage keeps ground)
    def inbox(p):
        return 60.5 < tomm(p.x) - O[0] < 66.5 and 28 < tomm(p.y) - O[1] < 38.6
    stubs = [t for t in b.GetTracks() if t.GetNetname() == "GND" and
             (inbox(t.GetPosition()) if t.Type() == pcbnew.PCB_VIA_T else inbox(t.GetStart()) and inbox(t.GetEnd()))]
    for t in stubs:
        b.Remove(t)
    print("removed", len(stubs), "ground tracks and vias of the old chip")
elif sys.argv[1] == "silk":
    # J12 and J3 on the right edge, pads running down from pin 1: names to the left, titles to the
    # right, on both sides (the back faces up in the head)
    for back in (False, True):
        for ref, names, size, dx, title, ty in (("J12", ("B+", "G", "5V", "G"), 0.8, -2.65, "EXT 5V", 32.6),
                                                ("J3", ("G", "3V3", "SDA", "SCL"), 0.8, -2.35, "I2C", 42.75)):
            for pad, name in zip(sorted(b.FindFootprintByReference(ref).Pads(), key=lambda p: int(p.GetNumber())), names):
                silk(name, tomm(pad.GetPosition().x) - O[0] + dx, tomm(pad.GetPosition().y) - O[1], size, back=back)
            silk(title, 73.15, ty, rot=90, back=back)
    imu_labels()
    # J11's pad names are in its footprint; its title beside it
    silk("EXT AMP", 66.75, 36.0, rot=90, back=True)
    # JP3's cut note, off the module's outline, above the jumper
    silk("CUT AMP", 64.5, 24.0)
    # the rev line of the name block, as gen_head_pcb.py puts it (the module centre, 2 mm rows)
    for d in b.GetDrawings():
        if d.Type() == pcbnew.PCB_TEXT_T and d.GetText() == "Dukki head":
            x, y = tomm(d.GetPosition().x) - O[0], tomm(d.GetPosition().y) - O[1]
            silk("rev B ext IMU", x, y + 2.0, 1.2, back=True)
    # the module on the front: which way round (pin 1 VCC, its square pad, at the CM4 end)
    silk("IMU MODULE", 57.3, 37.4)
else:
    raise SystemExit("stage: project, unsilk, silk or imusilk")
pcbnew.SaveBoard(str(PCB), b)
print("saved", PCB)
