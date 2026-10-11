#!/usr/bin/env python3
"""Generate the first board of the bridge carrier from its schematic netlist (docs/bridge-pcb.md).

One-shot bootstrap like gen_bridge_sch.py: once the board has been edited in KiCad, the
.kicad_pcb file is the source; the script refuses to overwrite it without --force.

Builds what KiCad's "Update PCB from Schematic" would (footprints linked to their symbols,
nets on every pad), then places the parts per the floor plan, draws the 45 x 80 mm outline
and sets the design rules. Board coordinates below are in mm from the board's top-left
corner, x right, y down; the board sits at ORIGIN on the page.

Runs on the KiCad VM, with the netlist exported first:
    kicad-cli sch export netlist -o /tmp/bridge.net hardware/bridge/bridge.kicad_sch
    python3 scripts/kicad/gen_bridge_pcb.py /tmp/bridge.net [--force]
"""
import argparse, re, sys
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("netlist")
ap.add_argument("--force", action="store_true")
ap.add_argument("--kicad-share", default="/usr/share/kicad")
args = ap.parse_args()
PROJ = Path("hardware/bridge")
OUT = PROJ / "bridge.kicad_pcb"
if OUT.exists() and not args.force:
    raise SystemExit(f"{OUT} exists; it is the source now. Use --force to overwrite it.")

W, H = 45.0, 80.0
ORIGIN = (100.0, 50.0)
FPLIBS = {"WeAct_STM32G474CoreBoard": Path("hardware/kicad/WeAct_STM32G474CoreBoard.pretty"),
          "LSM6DSV16X_Breakout": Path("hardware/kicad/LSM6DSV16X_Breakout.pretty")}

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
        "lcsc": (re.search(r"\(name \"LCSC\"\)\s*\(value \"([^\"]*)\"\)", block) or [None, ""])[1],
    }
pad_net = {}
for block in re.split(r"\n\t\t\(net\b", net_text.split("(nets", 1)[1])[1:]:
    name = re.search(r"\(name \"([^\"]+)\"\)", block).group(1)
    for ref, pin in re.findall(r"\(ref \"([^\"]+)\"\)\s*\(pin \"([^\"]+)\"\)", block):
        pad_net[(ref, pin)] = name

# ------------------------------------------------------------------ placement
# ref: (x, y, rotation in degrees, KiCad positive = counter-clockwise on screen)
PLACE = {
    # top band, everything tall. PH connectors: through hole, pin 1 at the origin, pins along +x
    "J6": (19.5, 3.6, 0),       # head, top row centre: the neck harness leaves upwards
    "J2": (9.0, 3.6, 0),        # segment A
    "J3": (32.0, 3.6, 0),       # segment B
    "J4": (3.65, 9.1, 270),     # segment C, turned so its pin row runs down the left edge, latch side out
    "J1": (20.0, 11.2, 0),      # XT30U-M, second row centre: pad 1 (GND, square), pad 2 (+BATT) 5 mm right
    "J5": (41.35, 13.1, 90),    # segment D, turned so its pin row runs up the right edge, latch side out
    # strip under the connectors: display and button connectors, main fuse
    "J7": (9.65, 10.8, 270),     # display, 4-pin PH, turned like SEG C and set beside it
    "J8": (15.5, 18.0, 0),      # shutdown button, 2-pin PH
    "F1": (31.6, 17.2, 270),    # main fuse, pad 1 (from the XT30) at the top
    # middle band: Q1 (turned so source and gate face the fuse column), branch fuses, trunk parts
    "Q1": (28.1, 24.6, 180),
    "D2": (37.6, 17.9, 0),      # input TVS, cathode (left) to F1 and Q1's source
    "F2": (41.0, 23.2, 0), "F3": (41.0, 25.8, 0), "F4": (41.0, 28.4, 0), "F5": (41.0, 31.0, 0), "F6": (41.0, 33.6, 0),
    "C3": (25.8, 31.6, 0), "C4": (30.8, 31.6, 0), "C5": (35.0, 31.6, 0), "C6": (35.0, 34.0, 0),
    # middle band, left: soft-start gate network by Q1, status LED, pack divider, data-line parts
    "D1": (19.9, 23.8, 0), "C1": (19.9, 26.4, 0), "R2": (19.9, 28.8, 0),
    "R1": (15.4, 28.8, 0), "C2": (15.4, 26.4, 0), "R3": (15.4, 23.8, 0),
    "R4": (4.0, 23.4, 0), "D3": (7.4, 23.4, 0),
    "R11": (4.0, 26.0, 0), "R12": (7.4, 26.0, 0), "C13": (10.8, 26.0, 0),
    "R5": (3.6, 28.8, 0), "D4": (7.2, 28.8, 0),
    "R6": (3.6, 31.2, 0), "D5": (7.2, 31.2, 0),
    "R7": (11.6, 31.2, 0), "D6": (15.2, 31.2, 0),
    "R8": (11.6, 33.6, 0), "D7": (15.2, 33.6, 0),
    # WeAct board flush, long axis across, USB-C at the left board edge
    "U2": (18.9, 49.6, 0),
    # LSM6DSV16X breakout flush at the bottom between the lower holes, pin row up towards the WeAct P1 row
    "U3": (22.5, 72.3, 180),
    # right strip beside the WeAct board: 5 V buck
    "C7": (39.2, 37.6, 90), "C8": (42.4, 37.6, 90),
    "U1": (40.8, 42.0, 0), "C10": (38.4, 45.0, 90), "C9": (43.0, 45.0, 90),
    "L1": (40.8, 49.4, 0),
    "C11": (39.2, 53.7, 90), "C12": (42.4, 53.7, 90),
    "R9": (39.2, 57.3, 90), "R10": (42.4, 57.3, 90),
    "D8": (40.8, 60.1, 180),    # buck output Schottky, cathode towards JP1 and the 5 V rail
    "JP1": (40.8, 62.5, 0),     # buck output to the 5 V rail: cut for an external step-down
    # backup 5 V wire pads, bottom right: 1 BAT+, 2 GND, 3 5V in, 4 GND, along +x
    "J9": (34.4, 67.8, 90),
    # corner holes
    "H1": (3.5, 3.5, 0), "H2": (41.5, 3.5, 0), "H3": (3.5, 76.5, 0), "H4": (41.5, 76.5, 0),
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
    nc.SetClearance(mm(0.2)); nc.SetTrackWidth(mm(0.25)); nc.SetViaDiameter(mm(0.6)); nc.SetViaDrill(mm(0.3))

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
    fp.SetSheetfile("bridge.kicad_sch") if hasattr(fp, "SetSheetfile") else None
    # carry the symbol's fields and flags so schematic parity holds and the BOM is right
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
    # Reference designators go on the fab layer: JLC places by the CPL file, and on a board this
    # dense they only collide. The silkscreen carries the labels needed by hand (below).
    ref_t.SetLayer(pcbnew.F_Fab)
    if ref == "J1":   # KiCad's library has no XT30U-M model: use ours (scripts/kicad/gen_3d_models.py)
        m = pcbnew.FP_3DMODEL()          # (editing fp.Models() entries in place only changes a copy)
        m.m_Filename = "${KIPRJMOD}/../kicad/3d/AMASS_XT30U-M.step"
        fp.Models().clear()
        fp.Models().append(m)
    x, y, rot = PLACE[ref]
    board.Add(fp)
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)
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

# silkscreen labels for hand assembly and plugging in
def silk(text, x, y, size=0.9, rot=0, left=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text); t.SetPosition(pt(x, y)); t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15))
    t.SetTextAngleDegrees(rot)
    if left:
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    board.Add(t)
def label_under(ref, text, dy=4.1):
    """Centre a label just below a PH connector (pin 1 at the footprint origin, pins along +x)."""
    fp = board.FindFootprintByReference(ref)
    n = len(fp.Pads())
    x = pcbnew.ToMM(fp.GetPosition().x) - ORIGIN[0] + (n - 1)
    y = pcbnew.ToMM(fp.GetPosition().y) - ORIGIN[1] + dy
    silk(text, x, y)
for ref, text in (("J2", "SEG A"), ("J3", "SEG B"), ("J6", "HEAD"), ("J8", "BTN")):
    label_under(ref, text)
silk("SEG C", 3.1, 16.6, 0.9)          # under the turned SEG C connector
silk("OLED", 9.1, 20.1, 0.9)           # under the turned display connector
silk("SEG D", 37.8, 11.1, 0.9, 90)
silk("BATT", 13.9, 11.2, 0.8)
silk("CUT FOR", 40.8, 64.3, 0.8)
silk("EXT 5V", 40.8, 65.4, 0.8)
for i, t in enumerate(("B+", "G", "5V", "G")):
    silk(t, 34.4 + 2.54 * i, 69.9, 0.8)
for i, t in enumerate(("Dukki bridge", "rev A 2026", "Lachlan Hurst")):     # left-aligned at the edge
    silk(t, 1.0, 66.2 + 1.5 * i, 0.9, left=True)
silk("WeAct STM32G474CEU6 CoreBoard", 18.9, 47.0, 1.0)    # inside the module outlines
silk("LSM6DSV16X breakout", 22.5, 72.6, 0.9)
# drill and placement origin at the board's lower-left corner: Gerbers, drill files and the
# placement file then share one origin (JLC flagged an offset when they did not)
board.GetDesignSettings().SetAuxOrigin(pt(0, H))
board.SetFileName(str(OUT.resolve()))
pcbnew.SaveBoard(str(OUT), board)
print("wrote", OUT, len(comps), "footprints,", len(nets), "nets")
