#!/usr/bin/env python3
"""Generate the first board of the head carrier from its schematic netlist (docs/head-pcb.md).

One-shot bootstrap like gen_bridge_pcb.py: once the board has been edited in KiCad, the
.kicad_pcb file is the source; the script refuses to overwrite it without --force.

Builds what KiCad's "Update PCB from Schematic" would (footprints linked to their symbols, nets
on every pad), places the parts per the floor plan, draws the 80 x 55 mm outline and sets the
design rules. Board coordinates are mm from the board's top-left corner, x right, y down, seen
from the component side (the side that faces down in the head); the board sits at ORIGIN.

Runs on the KiCad VM, with the netlist exported first:
    kicad-cli sch export netlist -o /tmp/head.net hardware/head/head.kicad_sch
    python3 scripts/kicad/gen_head_pcb.py /tmp/head.net [--force]
"""
import argparse, re
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("netlist")
ap.add_argument("--force", action="store_true")
ap.add_argument("--kicad-share", default="/usr/share/kicad")
args = ap.parse_args()
PROJ = Path("hardware/head")
OUT = PROJ / "head.kicad_pcb"
if OUT.exists() and not args.force:
    raise SystemExit(f"{OUT} exists; it is the source now. Use --force to overwrite it.")

W, H = 80.0, 55.0
ORIGIN = (100.0, 50.0)
FPLIBS = {"DukkiHead": Path("hardware/kicad/DukkiHead.pretty")}

def mm(v):
    return pcbnew.FromMM(v)

def pt(x, y):
    return pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))

# ------------------------------------------------------------------ netlist
net_text = Path(args.netlist).read_text()
comps = {}
for block in re.split(r"\n\t\t\(comp\b", net_text.split("(components", 1)[1].split("\n\t(libparts", 1)[0])[1:]:
    ref = re.search(r"\(ref \"([^\"]+)\"\)", block).group(1)
    comps[ref] = {
        "value": re.search(r"\(value \"([^\"]*)\"\)", block).group(1),
        "footprint": re.search(r"\(footprint \"([^\"]*)\"\)", block).group(1),
        "tstamp": re.findall(r"\(tstamps \"([^\"]+)\"\)", block)[-1],
        "dnp": bool(re.search(r'\(property \(name "dnp"\)', block)),
        "no_bom": bool(re.search(r'\(property \(name "exclude_from_bom"\)', block)),
        "fields": dict(re.findall(r'\(field\s*\(name "([^"]+)"\)\s*"([^"]*)"\)', block)),
    }
pad_net = {}
for block in re.split(r"\n\t\t\(net\b", net_text.split("(nets", 1)[1])[1:]:
    name = re.search(r"\(name \"([^\"]+)\"\)", block).group(1)
    for ref, pin in re.findall(r"\(ref \"([^\"]+)\"\)\s*\(pin \"([^\"]+)\"\)", block):
        pad_net[(ref, pin)] = name

# ------------------------------------------------------------------ placement
# The module (H5) fills the left end: 40 x 55, centred at (26, 27.5), flush with the top and bottom
# edges. Its sockets sit at fixed offsets from its centre (hardware/kicad/README.md). The free
# block is x 46.6 to 80. A strip 5.7 mm wide is left of the module for three PH headers.
MOD = (26.0, 27.5)
SOCK = {"J9": (-16.96, 2.5), "J10": (16.96, 2.5)}
PLACE = {
    "H5": (MOD[0], MOD[1], 0),
    "J9": (MOD[0] + SOCK["J9"][0], MOD[1] + SOCK["J9"][1], 0),
    "J10": (MOD[0] + SOCK["J10"][0], MOD[1] + SOCK["J10"][1], 0),
    # corner holes, M2, centres 3 mm in
    "H1": (3.0, 3.0, 0), "H2": (77.0, 3.0, 0), "H3": (3.0, 52.0, 0), "H4": (77.0, 52.0, 0),
    # left strip, beside the I2C8 and SAI2 pins of J9: PH headers turned 270 (pin 1 at the top,
    # pins running down), courtyard x 0.1 to 5.7
    "J7": (3.45, 8.6, 270),     # microphone, 5 pins
    "J2": (3.45, 23.1, 270),    # ToF, 4 pins
    "J3": (3.45, 40.6, 270),    # I2C breakout, 4 pins: below the module's 5 V pins (y 35.4 to 37.4)
    # right column, courtyard x 74.1 to 79.7: head bundle at the top, then fan, speaker, eye LED
    "J1": (77.45, 8.6, 270),
    "J6": (77.45, 19.7, 270),
    "J5": (77.45, 26.8, 270),
    "J4": (77.45, 33.9, 270),
    # camera FPC on the bottom edge, centred in the free block, turned so its contact pads (the
    # cable entry side, per XUNPU's drawing) face the edge
    "J8": (63.0, 52.25, 180),
    # buck, top of the free block, beside the head bundle
    # U2 turned so VIN and FB face the input side, SW and CB face L1; L1 turned so its switch-node
    # pad faces U2 and its 5 V pad the output capacitors (pin 1 at the top)
    "L1": (55.6, 4.2, 180),
    "U2": (63.0, 3.0, 180),
    "C5": (63.0, 6.3, 0),
    "D1": (71.6, 4.3, 90),
    "C1": (67.5, 3.3, 270),
    "C4": (66.4, 6.5, 0),
    "C2": (67.6, 9.7, 0),
    "C3": (71.0, 10.2, 90),
    "R1": (63.0, 8.6, 0), "R2": (63.0, 10.4, 0),
    "C6": (52.4, 10.4, 270), "C7": (55.0, 10.4, 270), "C8": (57.6, 10.4, 270),
    # 3.3 V
    "U3": (55.6, 18.8, 0),
    "C9": (53.4, 23.6, 0),                      # input, by VI (pin 3, bottom of the pin column)
    "C10": (49.6, 18.4, 90), "C11": (47.6, 18.4, 90),   # output, left of VO (pin 2)
    # amplifier
    "U4": (63.6, 16.0, 0),
    "C12": (63.2, 20.0, 0), "C13": (63.2, 22.3, 0),
    "R3": (67.0, 15.5, 90), "R4": (67.0, 19.2, 90),
    # fan
    "Q1": (71.5, 15.2, 0),
    "D2": (69.0, 21.0, 90),
    "R5": (72.4, 19.6, 90), "R6": (72.4, 23.2, 90),
    # eye LED data resistor, power LED, I2C8 pull-up footprints (not fitted)
    "R7": (72.9, 31.0, 90),
    "R12": (66.4, 25.2, 0), "D3": (69.6, 25.2, 0),
    "R8": (58.0, 26.2, 0), "R9": (61.4, 26.2, 0),
    # BMI088 module, rows across, 7-pin row towards the bottom
    "U5": (53.87, 41.51, 0),
    # camera I2C pull-ups and decoupling, left of the FPC
    "R10": (47.6, 51.7, 90), "R11": (49.4, 51.7, 90), "C14": (51.2, 51.7, 90),
}

# ------------------------------------------------------------------ build
board = pcbnew.BOARD()
ds = board.GetDesignSettings()
ds.SetCopperLayerCount(2)
ds.m_TrackMinWidth = mm(0.15)
ds.m_ViasMinSize = mm(0.5)
ds.m_MinThroughDrill = mm(0.3)
ds.m_CopperEdgeClearance = mm(0.3)
nc = ds.m_NetSettings.GetDefaultNetclass() if hasattr(ds.m_NetSettings, "GetDefaultNetclass") else None
if nc is not None:
    nc.SetClearance(mm(0.15)); nc.SetTrackWidth(mm(0.2)); nc.SetViaDiameter(mm(0.6)); nc.SetViaDrill(mm(0.3))

nets = {}
def net(name):
    if name not in nets:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
        nets[name] = n
    return nets[name]

def lib_path(lib):
    return str(FPLIBS.get(lib, Path(args.kicad_share) / "footprints" / f"{lib}.pretty"))

missing = [r for r in comps if r not in PLACE]
if missing:
    raise SystemExit(f"no placement for {missing}")
for ref, c in sorted(comps.items()):
    lib, name = c["footprint"].split(":")
    fp = pcbnew.FootprintLoad(lib_path(lib), name)
    if fp is None:
        raise SystemExit(f"{ref}: footprint {c['footprint']} not found")
    fp.SetReference(ref)
    fp.SetValue(c["value"])
    fp.SetFPIDAsString(c["footprint"]) if hasattr(fp, "SetFPIDAsString") else fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetPath(pcbnew.KIID_PATH(f"/{c['tstamp']}"))
    fp.SetSheetname("Root") if hasattr(fp, "SetSheetname") else None
    fp.SetSheetfile("head.kicad_sch") if hasattr(fp, "SetSheetfile") else None
    fp.GetField("Datasheet").SetText(c["fields"].get("Datasheet", ""))
    for k, v in c["fields"].items():
        if k in ("Footprint",):
            continue
        if fp.HasField(k):
            fp.GetField(k).SetText(v)
        else:
            fp.SetField(k, v)
        fp.GetField(k).SetVisible(False)
    fp.SetDNP(c["dnp"])
    fp.SetExcludedFromBOM(c["no_bom"])
    fp.Value().SetVisible(False)
    fp.Value().SetLayer(pcbnew.F_Fab)
    ref_t = fp.Reference()
    ref_t.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
    ref_t.SetTextThickness(mm(0.12))
    ref_t.SetLayer(pcbnew.F_Fab)      # as on the bridge: JLC places by the CPL, labels below
    x, y, rot = PLACE[ref]
    board.Add(fp)
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)
    if ref in ("H5", "J9", "J10"):
        fp.SetLocked(True)            # the sockets' offsets from the module are fixed
    for pad in fp.Pads():
        n = pad_net.get((ref, pad.GetNumber()))
        if n:
            pad.SetNet(net(n))

# outline, 1 mm corner radius
R = 1.0
def seg(a, b):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pt(*a)); s.SetEnd(pt(*b)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
def arc(c, a, b):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
    s.SetCenter(pt(*c)); s.SetStart(pt(*a)); s.SetEnd(pt(*b)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
seg((R, 0), (W - R, 0)); seg((W, R), (W, H - R)); seg((W - R, H), (R, H)); seg((0, H - R), (0, R))
arc((R, R), (0, R), (R, 0)); arc((W - R, R), (W - R, 0), (W, R))
arc((W - R, H - R), (W, H - R), (W - R, H)); arc((R, H - R), (R, H), (0, H - R))

# silkscreen. The component side faces down in the head and the module covers most of it, so the
# connector names go on the back silkscreen, which faces up, beside each header's pins; text on the
# back reads mirrored from the front, as KiCad draws it.
def silk(text, x, y, size=1.0, rot=0, back=False, left=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text); t.SetPosition(pt(x, y)); t.SetLayer(pcbnew.B_SilkS if back else pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15))
    t.SetTextAngleDegrees(rot)
    t.SetMirrored(back)
    if left:
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    board.Add(t)
def label_beside(ref, text, dx):
    """Name a PH header turned 270 (pins running down from pin 1), dx from its pin column."""
    fp = board.FindFootprintByReference(ref)
    n = len(fp.Pads())
    x = pcbnew.ToMM(fp.GetPosition().x) - ORIGIN[0] + dx
    y = pcbnew.ToMM(fp.GetPosition().y) - ORIGIN[1] + (n - 1)
    silk(text, x, y, 1.0, 90, back=True)
    silk("1", x, pcbnew.ToMM(fp.GetPosition().y) - ORIGIN[1] - 1.9, 0.8, 0, back=True)
for ref, text in (("J7", "MIC"), ("J2", "TOF"), ("J3", "I2C")):
    label_beside(ref, text, -2.1)
for ref, text in (("J1", "HEAD"), ("J6", "FAN"), ("J5", "SPK"), ("J4", "EYE")):
    label_beside(ref, text, -2.0)
silk("Dukki head rev A 2026", 63.0, 47.4, 1.0, back=True)
silk("Lachlan Hurst", 63.0, 45.9, 1.0, back=True)
silk("CAM", 63.0, 47.6, 0.8)
board.GetDesignSettings().SetAuxOrigin(pt(0, H))
board.SetFileName(str(OUT.resolve()))
pcbnew.SaveBoard(str(OUT), board)
print("wrote", OUT, len(comps), "footprints,", len(nets), "nets")
