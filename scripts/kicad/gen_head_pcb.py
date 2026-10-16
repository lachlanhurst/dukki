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
        "dnp": bool(re.search(r'\(name "dnp"\)', block)),
        "no_bom": bool(re.search(r'\(name "exclude_from_bom"\)', block)),
        "fields": dict(re.findall(r'\(field\s*\(name "([^"]+)"\)\s*"([^"]*)"\)', block)),
    }
pad_net = {}
for block in re.split(r"\n\t\t\(net\b", net_text.split("(nets", 1)[1])[1:]:
    name = re.search(r"\(name \"([^\"]+)\"\)", block).group(1)
    for ref, pin in re.findall(r"\(ref \"([^\"]+)\"\)\s*\(pin \"([^\"]+)\"\)", block):
        pad_net[(ref, pin)] = name

# ------------------------------------------------------------------ placement
# The module (H5) fills the left end: 40 x 55, centred at (26, 27.5), flush with the top and bottom
# edges, turned 180 degrees so its first socket (J9: 5 V, GPIO, the SAI2, UART and I2C pins) faces
# the free block and its second (J10, the camera lanes) faces the left edge. The sockets sit at
# fixed offsets from the module's centre (hardware/kicad/README.md), turned with it. The camera
# pairs leave J10 westwards, run down the left strip and east under the module's lower edge to J8.
MOD = (26.0, 27.5)
SOCK = {"J9": (16.96, -2.5), "J10": (-16.96, -2.5)}       # (+-16.96, 2.5) turned 180 degrees
PLACE = {
    "H5": (MOD[0], MOD[1], 180),
    "J9": (MOD[0] + SOCK["J9"][0], MOD[1] + SOCK["J9"][1], 180),
    "J10": (MOD[0] + SOCK["J10"][0], MOD[1] + SOCK["J10"][1], 180),
    # corner holes, M2, centres 3 mm in
    "H1": (3.0, 3.0, 0), "H2": (77.0, 3.0, 0), "H3": (3.0, 52.0, 0), "H4": (77.0, 52.0, 0),
    # left strip above the camera lanes, courtyard x 0.1 to 5.7: PH headers turned 270 (pin 1 at
    # the top, pins running down), fed from J9's inner row round the socket's top end
    # pin column at x 3.15: the housings end about 0.65 mm short of the module's edge (x 5.95)
    "J2": (3.15, 8.6, 270),     # ToF, 4 pins
    "J4": (3.15, 19.7, 270),    # eye LED, 3 pins
    # right column, courtyard x 74.1 to 79.7: head bundle at the top, then microphone, speaker, fan
    "J1": (77.45, 8.6, 270),
    "J7": (77.45, 19.7, 270),
    "J5": (77.45, 32.8, 270),
    "J6": (77.45, 39.9, 270),
    # spare I2C header in the free block, beside the BMI088
    "J3": (71.4, 32.0, 270),
    # camera FPC on the bottom edge, centred in the free block, turned so its contact pads (the
    # cable entry side, per XUNPU's drawing) face the edge
    "J8": (63.0, 52.25, 180),
    # buck, top of the free block, beside the head bundle. U2 turned so VIN and GND face the top
    # edge, with the 100 nF C4 straight across them and ground vias beside its GND pin (the tight input
    # loop); SW and CB face L1, the boot cap C5 beside them; FB faces the divider below. L1 turned so
    # its switch-node pad faces U2, its 5 V pad the output capacitors.
    "L1": (55.3, 4.2, 180),
    "U2": (62.8, 4.0, 180),
    "C4": (62.8, 1.35, 180),            # pad 1 (VIN) right, pad 2 (GND) left, over U2's pins 3 and 4
    "C5": (59.85, 4.5, 90),             # pad 2 (SW) up beside U2's SW pin, pad 1 (CB) down beside CB
    "R1": (63.2, 6.6, 0), "R2": (63.2, 8.4, 180),   # FB pads at the right, under U2's FB pin
    # input chain in current order down the right: J1, the TVS (cathode beside J1 pin 1), the bulk
    # capacitor, then the two 10 uF ceramics beside U2
    "C1": (66.7, 3.3, 270), "C2": (70.15, 3.3, 270),
    "D1": (70.3, 8.0, 180),
    "C17": (70.45, 15.05, 270),         # 47 uF electrolytic, + at the top
    "C6": (52.4, 10.4, 270), "C7": (55.0, 10.4, 270), "C8": (57.6, 10.4, 270),
    # 3.3 V: pins down the left side (GND, VO, VI), tab right; capacitors in a row below
    "U3": (55.6, 18.8, 0),
    "C9": (53.4, 23.6, 0), "C10": (57.2, 23.6, 0), "C11": (60.4, 23.6, 0),
    # amplifier: SD_MODE resistor left of it, GAIN resistor (not fitted) above
    "U4": (64.6, 16.0, 0),
    "C12": (64.2, 20.0, 0), "C13": (64.2, 22.3, 0),
    "R3": (61.3, 19.6, 90), "R4": (62.8, 12.6, 0),
    # fan
    "Q1": (71.9, 21.7, 0),
    "D2": (68.2, 22.2, 90),
    "R5": (71.9, 24.3, 0), "R6": (71.9, 26.1, 0),
    # eye LED data resistor, power LED, I2C8 pull-up footprints (not fitted)
    "R7": (59.8, 13.4, 270),
    "R12": (48.4, 2.0, 0), "D3": (48.4, 4.6, 0),   # power LED in the free top-left corner
    # the 5 V link from the buck (cut for an external 5 V), in the 5 V run west of the buck
    "JP1": (48.6, 12.4, 270),          # pad 1 (buck side) at the top
    # the amplifier's 5 V link (cut for an external amplifier module), below the amplifier
    "JP3": (64.0, 26.5, 0),
    # BMI088 in the middle of the free block, away from the mounting holes and the buck; its
    # decoupling, the I2C8 pull-ups and the two links that isolate it for an external module
    "U5": (63.6, 34.0, 0),
    "C15": (63.6, 29.6, 180), "C16": (63.6, 37.4, 0),   # C15 turned: its 3.3 V pad to the right
    "R8": (54.0, 27.0, 0), "R9": (54.0, 28.8, 0),
    "JP4": (59.3, 29.4, 90), "JP5": (59.3, 33.2, 90),
    # fallback wire pads, 2.54 mm, rows across the bottom of the free block, pin 1 at the left:
    # external 5 V (B+, GND, 5V, GND) and the external amplifier module (LRC, BCLK, DIN, GAIN, SD,
    # GND, Vin)
    "J12": (48.6, 37.4, 90),
    "J11": (48.6, 41.6, 90),
    # SPI breakout wire pads, on the bottom under the module along the top edge (the camera pairs run
    # along the left and lower edges): pad 1 (GND) at the right, so they read left to right from the back
    "J13": (35.82, 2.6, 0, "back"),    # pads over 6 mm from both M2.5 module holes
    # link UART series resistors just outside the module edge, where UART7 leaves J9 for the bottom
    "R14": (48.4, 25.6, 0), "R13": (48.4, 27.2, 0),
    # reset and maskrom test pads, bottom, under the module beside J9's pins 92 and 93
    "TP1": (38.5, 11.0, 0, "back"), "TP2": (41.5, 11.0, 0, "back"), "TP3": (44.5, 11.0, 0, "back"),
    # USB download port pads, bottom, under the module beside J10's pins 103 and 105: D+, D-, G top to bottom
    "TP5": (14.0, 33.3, 0, "back"), "TP4": (14.0, 35.8, 0, "back"), "TP6": (14.0, 38.3, 0, "back"),
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
    x, y, rot = PLACE[ref][:3]
    board.Add(fp)
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)
    if len(PLACE[ref]) > 3 and PLACE[ref][3] == "back":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT if hasattr(pcbnew, "FLIP_DIRECTION_LEFT_RIGHT") else False)
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
for ref, text in (("J2", "TOF"), ("J4", "EYE"), ("J3", "I2C/IMU")):
    label_beside(ref, text, -2.1)
for ref, text in (("J1", "HEAD"), ("J7", "MIC"), ("J5", "SPK"), ("J6", "FAN")):
    label_beside(ref, text, -2.0)
# name block, centred on the back over the module, as on the bridge
for i, t in enumerate(("Dukki head", "rev B 2026", "Lachlan Hurst")):
    silk(t, MOD[0], MOD[1] - 2.0 + 2.0 * i, 1.2, back=True)
# front: the camera connector, the fallback wire pads and the links to cut for them
silk("CAM", 63.0, 47.6, 0.8)
# camera FPC ends: the dual-contact socket takes the cable either way up, and upside down reverses
# it, so mark both ends; check 3V3 to the camera's 3V3 with a meter before first power
j8 = board.FindFootprintByReference("J8")
for num, name in (("1", "1 GND"), ("15", "15 3V3")):
    p = j8.FindPadByNumber(num)
    silk(name, pcbnew.ToMM(p.GetPosition().x) - ORIGIN[0], 47.6, 0.8)
for ref, name in (("TP1", "RST"), ("TP2", "BOOT"), ("TP3", "G")):
    fp = board.FindFootprintByReference(ref)
    silk(name, pcbnew.ToMM(fp.GetPosition().x) - ORIGIN[0], pcbnew.ToMM(fp.GetPosition().y) - ORIGIN[1] + 1.8, 0.8, back=True)
for ref, name in (("TP5", "D+"), ("TP4", "D-"), ("TP6", "G")):
    fp = board.FindFootprintByReference(ref)
    silk(name, pcbnew.ToMM(fp.GetPosition().x) - ORIGIN[0] + 2.0, pcbnew.ToMM(fp.GetPosition().y) - ORIGIN[1], 0.8, back=True)
silk("USB", 14.0, 31.4, 0.8, back=True)
def pad_labels(ref, names, dy, size=0.8):
    fp = board.FindFootprintByReference(ref)
    for pad in fp.Pads():
        x = pcbnew.ToMM(pad.GetPosition().x) - ORIGIN[0]
        y = pcbnew.ToMM(pad.GetPosition().y) - ORIGIN[1]
        silk(names[int(pad.GetNumber()) - 1], x, y + dy, size)
pad_labels("J12", ("B+", "G", "5V", "G"), -2.3)
silk("EXT 5V", 53.3, 33.6, 0.8)
pad_labels("J11", ("LR", "BC", "DI", "GA", "SD", "G", "5V"), 2.35, 0.8)
silk("EXT AMP", 62.0, 39.3, 0.8)
silk("CUT: EXT 5V", 48.6, 18.9, 0.8, 90)
silk("CUT: AMP", 67.3, 28.0, 0.8, 90)
silk("CUT: EXT IMU", 57.4, 31.3, 0.8, 90)
board.GetDesignSettings().SetAuxOrigin(pt(0, H))
board.SetFileName(str(OUT.resolve()))
pcbnew.SaveBoard(str(OUT), board)
print("wrote", OUT, len(comps), "footprints,", len(nets), "nets")
