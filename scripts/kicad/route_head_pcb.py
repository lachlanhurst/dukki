#!/usr/bin/env python3
"""Route the head carrier board produced by gen_head_pcb.py (one-shot bootstrap, like it).

1. Net classes: 5 V and the buck input 0.6 mm, the buck switch node 0.8 mm, 3.3 V 0.4 mm,
   signals 0.2 mm.
2. Drawn by hand and locked: the three camera pairs from J10 to J8 along the corridor kept clear
   for them (docs/head-pcb.md section 8), and the 5 V trunk from the buck to the module's 5 V pins,
   on both layers.
3. Everything else autorouted by Freerouting (Specctra DSN out, SES back in), with the bottom
   layer under the camera pairs kept free of tracks so their ground reference is unbroken.
4. Ground pours on both layers, a 5 V pad over the module's 5 V pins, stitching vias.

Runs on the KiCad VM after gen_head_pcb.py:
    python3 scripts/kicad/route_head_pcb.py [--passes 40]
Freerouting and a Java runtime are expected under ~/opt (see docs/bridge-pcb.md).
"""
import argparse, os, re, shutil, subprocess
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("--passes", type=int, default=40)
ap.add_argument("--hand-only", action="store_true", help="stop after the hand-drawn copper (to check it)")
ap.add_argument("--tries", type=int, default=3, help="Freerouting runs; the one with fewest unrouted is kept")
ap.add_argument("--java", default=os.path.expanduser("~/opt/jre/bin/java"))
ap.add_argument("--freerouting", default=os.path.expanduser("~/opt/freerouting/freerouting-2.5.0.jar"))
args = ap.parse_args()

PCB = Path("hardware/head/head.kicad_pcb")
ORIGIN = (100.0, 50.0)
W, H = 80.0, 55.0
board = pcbnew.LoadBoard(str(PCB))

def drop_duplicate_vias(board):
    """Freerouting hands the locked vias back in its session file and the import adds them again,
    at the net class's via size: where two vias share a spot, keep the smaller, the original."""
    seen = {}
    for v in [v for v in board.GetTracks() if v.Type() == pcbnew.PCB_VIA_T]:
        k = (v.GetPosition().x, v.GetPosition().y)
        if k in seen:
            keep, drop = sorted((seen[k], v), key=lambda x: x.GetWidth(pcbnew.F_Cu))
            board.Remove(drop)
            seen[k] = keep
        else:
            seen[k] = v


def mm(v):
    return pcbnew.FromMM(v)

def pt(x, y):
    return pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))

def pad_xy(ref, num):
    p = board.FindFootprintByReference(ref).FindPadByNumber(num).GetPosition()
    return pcbnew.ToMM(p.x) - ORIGIN[0], pcbnew.ToMM(p.y) - ORIGIN[1]

def full(name):
    """Schematic-local nets carry the root sheet prefix ("/CSI_D0_N"); power-symbol nets do not."""
    return name if board.FindNet(name) else "/" + name

def net(name):
    n = board.FindNet(full(name))
    if n is None:
        raise SystemExit(f"no net {name!r}")
    return n

# ------------------------------------------------------------------ net classes
ns = board.GetDesignSettings().m_NetSettings
default = ns.GetDefaultNetclass()
default.SetClearance(mm(0.15)); default.SetTrackWidth(mm(0.2)); default.SetViaDiameter(mm(0.6)); default.SetViaDrill(mm(0.3))
def netclass(name, width, clearance, via=(0.8, 0.4), members=()):
    nc = pcbnew.NETCLASS(name)
    nc.SetTrackWidth(mm(width)); nc.SetClearance(mm(clearance))
    nc.SetViaDiameter(mm(via[0])); nc.SetViaDrill(mm(via[1]))
    ns.SetNetclass(name, nc)
    for m in members:
        ns.SetNetclassPatternAssignment(full(m), name)
netclass("Power", 0.6, 0.2, members=["VBAT_IN", "+5V", "5V_BUCK", "AMP_5V"])
netclass("Ground", 0.6, 0.15, members=["GND"])      # 0.15 mm: the BMI088's ground vias sit between 3.3 V stubs
netclass("Switch", 0.8, 0.2, members=["BUCK_SW"])
netclass("Rail3V3", 0.4, 0.15, members=["+3V3"])
netclass("CSI", 0.2, 0.2, members=["CSI_D0_N", "CSI_D0_P", "CSI_D1_N", "CSI_D1_P", "CSI_C_N", "CSI_C_P"])
ns.RecomputeEffectiveNetclasses() if hasattr(ns, "RecomputeEffectiveNetclasses") else None

# ------------------------------------------------------------------ hand-drawn copper
def track(points, netname, width, layer=pcbnew.F_Cu):
    for a, b in zip(points, points[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pt(*a)); t.SetEnd(pt(*b)); t.SetWidth(mm(width)); t.SetLayer(layer)
        t.SetNet(net(netname)); t.SetLocked(True)
        board.Add(t)

def via(x, y, netname, d=0.8, drill=0.4):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pt(x, y)); v.SetWidth(mm(d)); v.SetDrill(mm(drill))
    v.SetNet(net(netname)); v.SetLocked(True)
    board.Add(v)

# Camera pairs: 0.2 mm tracks 0.2 mm apart (0.4 mm pitch, the socket's own), on the top layer over
# the bottom plane (kept free of tracks beneath them), ground pour either side. Each pair leaves
# J10's outer row westwards, turns south down the left strip, east along the module's lower edge
# and below the BMI088, and south into J8. N starts south of P, on the left of the direction of
# travel, and stays on the left, so it arrives east of P as J8's pinout has it; the pairs nest
# without crossing, D0 innermost.
#   (pair, socket pin N, J8 pin N, south-run x, east-run y, final south-run x)
PAIRS = [("D0", "128", "2", 6.6, 44.6, 68.5),
         ("D1", "134", "5", 5.4, 46.2, 65.5),
         ("C", "140", "8", 4.2, 47.8, 62.5)]
JOG = 51.6                                    # where each pair leaves its pitch for J8's 1 mm pitch

def offset(poly, o):
    """The polyline moved o mm to the left of its direction of travel, mitred at right angles."""
    def normal(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = (dx * dx + dy * dy) ** 0.5
        return dy / n, -dx / n                # left of travel, y down
    out = []
    for k, q in enumerate(poly):
        ns = []
        if k > 0:
            ns.append(normal(poly[k - 1], q))
        if k < len(poly) - 1:
            ns.append(normal(q, poly[k + 1]))
        nx, ny = (ns[0] if len(ns) == 1 else (ns[0][0] + ns[1][0], ns[0][1] + ns[1][1]))
        out.append((q[0] + o * nx, q[1] + o * ny))
    return out

CSI_LINES = []
for name, sp, jp, xs, ye, xf in PAIRS:
    pn, pp = pad_xy("J10", sp), pad_xy("J10", str(int(sp) + 2))
    jn, jpp = pad_xy("J8", jp), pad_xy("J8", str(int(jp) + 1))
    mid = (pn[0], (pn[1] + pp[1]) / 2)
    centre = [mid, (xs, mid[1]), (xs, ye), (xf, ye), (xf, JOG)]
    CSI_LINES.append(centre)
    n_pts, p_pts = offset(centre, 0.2), offset(centre, -0.2)
    track(n_pts + [(jn[0], JOG + 0.8), jn], f"CSI_{name}_N", 0.2)
    track(p_pts + [(jpp[0], JOG + 0.8), jpp], f"CSI_{name}_P", 0.2)

# 5 V: L1's output pad and the three output capacitors joined, then west and south to a small
# locked copper area (below) over the module's six 5 V pads (77 to 87, J9's outer row, facing the
# free block).
l2 = pad_xy("L1", "2")
cs = [pad_xy(r, "1") for r in ("C6", "C7", "C8")]
yb = cs[0][1]
track([l2, (cs[0][0], yb)], "5V_BUCK", 1.5)
track([(cs[0][0], yb), (cs[-1][0], yb)], "5V_BUCK", 1.5)
p77, p87 = pad_xy("J9", "77"), pad_xy("J9", "87")
y5 = (p77[1] + p87[1]) / 2
X5 = 48.6
# through JP1, the link to cut for an external 5 V (J12); the buck's feedback divider takes its top
# from the buck side, so a cut link leaves the buck regulating its own capacitors
jp = [pad_xy("JP1", "1"), pad_xy("JP1", "2")]  # 1: buck side, at the top; 2: the 5 V rail
track([(cs[0][0], yb), (X5, yb), jp[0]], "5V_BUCK", 1.5)
track([jp[1], (X5, y5), (p77[0] + 1.8, y5)], "+5V", 1.5)
vi = pad_xy("U3", "3")                        # the 3.3 V regulator's input, off the same run
track([(X5, y5), (X5, vi[1]), vi], "+5V", 0.8)

# The eye LED header (J4) sits in the left strip: its 5 V and data cross under the module on the
# bottom layer, clear of the camera pairs, from vias by the 5 V run and by R7.
j41, j42 = pad_xy("J4", "1"), pad_xy("J4", "2")
r7 = pad_xy("R7", "2")                        # EYE_DIN end, turned to the bottom
ve = (r7[0], r7[1] + 1.4)
v5 = (X5 - 1.9, 16.2)                         # below JP1, north of the eye data's run
track([r7, ve], "EYE_DIN", 0.25)
via(*ve, "EYE_DIN", d=0.6, drill=0.3)
track([(X5, v5[1]), v5], "+5V", 0.6)
via(*v5, "+5V", d=0.6, drill=0.3)
track([v5, (j41[0] + 2.2, v5[1]), (j41[0] + 2.2, j41[1]), j41], "+5V", 0.5, pcbnew.B_Cu)
track([ve, (ve[0], 17.0), (j42[0] + 3.2, 17.0), (j42[0] + 3.2, j42[1]), j42], "EYE_DIN", 0.25, pcbnew.B_Cu)

# UART7 and the SAI2 bit clock leave J9's outer row through vias beside it and cross the free block
# on the bottom layer, under the parts that wall the top layer in. RX and TX swap order between J9
# and J1, so TX comes up to the top for its last few millimetres; SCLK comes up near J7 and has a
# branch up to the amplifier's BCLK.
def drop(pin, via_at, netname):
    a = pad_xy("J9", pin)
    track([a, (a[0] + 0.9, a[1]), via_at], netname, 0.2)
    via(*via_at, netname, d=0.6, drill=0.3)
# UART7 passes through its 330R series resistors (R14 TX, R13 RX) just outside the module edge;
# the vias to the bottom are on the J1 side of them (nets LINK_TX, LINK_RX).
VS = (45.6, 24.4)
drop("49", VS, "SAI2_SCLK")
r14, r13 = (pad_xy("R14", "1"), pad_xy("R14", "2")), (pad_xy("R13", "1"), pad_xy("R13", "2"))
p47, p45 = pad_xy("J9", "47"), pad_xy("J9", "45")
track([p47, (p47[0] + 0.9, p47[1]), r14[0]], "UART7_TX", 0.2)
track([p45, (p45[0] + 0.9, p45[1]), (p45[0] + 2.1, r13[0][1]), r13[0]], "UART7_RX", 0.2)
VT, VR = (r14[1][0] + 1.0, r14[1][1]), (r13[1][0] + 1.0, r13[1][1])
track([r14[1], VT], "LINK_TX", 0.2)
via(*VT, "LINK_TX", d=0.6, drill=0.3)
track([r13[1], VR], "LINK_RX", 0.2)
via(*VR, "LINK_RX", d=0.6, drill=0.3)
j13, j14, j73 = pad_xy("J1", "3"), pad_xy("J1", "4"), pad_xy("J7", "3")
u16 = pad_xy("U4", "16")
XR, XT_ = 75.8, 74.8
track([VR, (VR[0] + 0.2, VR[1] + 0.2), (XR, VR[1] + 0.2), (XR, j13[1] + 0.6), (XR + 0.6, j13[1]), j13],
      "LINK_RX", 0.2, pcbnew.B_Cu)
vt2 = (XT_ - 0.2, j14[1] + 1.8)
track([VT, (XT_ - 0.2, VT[1]), vt2], "LINK_TX", 0.2, pcbnew.B_Cu)
via(*vt2, "LINK_TX", d=0.6, drill=0.3)
track([vt2, (XT_ + 0.6, j14[1]), j14], "LINK_TX", 0.2)
vs2 = (73.9, VS[1])          # clear of the fan gate resistor R5
track([VS, vs2], "SAI2_SCLK", 0.2, pcbnew.B_Cu)
via(*vs2, "SAI2_SCLK", d=0.6, drill=0.3)
track([vs2, (j73[0] - 1.4, j73[1]), j73], "SAI2_SCLK", 0.2)
vb = (u16[0] - 0.35, u16[1] - 0.95)
track([(vb[0], VS[1]), vb], "SAI2_SCLK", 0.2, pcbnew.B_Cu)
via(*vb, "SAI2_SCLK", d=0.6, drill=0.3)
track([vb, u16], "SAI2_SCLK", 0.2)

# Buck switch node and bootstrap: short and wide, not left to the autorouter. SW from U2 straight
# to L1 with a branch down to C5; CB from U2 round under its FB pin to C5's other pad.
sw, cb, l1s = pad_xy("U2", "5"), pad_xy("U2", "6"), pad_xy("L1", "1")
c5s, c5b = pad_xy("C5", "2"), pad_xy("C5", "1")
xj = l1s[0] + 1.6
track([sw, (l1s[0], sw[1])], "BUCK_SW", 0.8)          # into L1's pad, past the C5 branch
track([(xj, sw[1]), (xj, c5s[1]), c5s], "BUCK_SW", 0.5)
track([cb, c5b], "BUCK_CB", 0.3)
# Input loop: C4 straight across U2's VIN and GND pins, and U2's GND to two vias beside it, so the
# hot loop closes in a few millimetres on the top layer and drops to the bottom plane at once.
c4v, c4g = pad_xy("C4", "1"), pad_xy("C4", "2")
u2v, u2g = pad_xy("U2", "3"), pad_xy("U2", "4")
track([c4v, u2v], "VBAT_IN", 0.5)
track([c4g, u2g], "GND", 0.5)
# USB download port: J10's pins 105 (D+) and 103 (D-) on the inner row straight out to a via each,
# then on the bottom to their test pads (TP5, TP4)
usb = ((pad_xy("J10", "105"), (11.9, 34.0), pad_xy("TP5", "1"), "USB_DP"),
       (pad_xy("J10", "103"), (12.3, 34.9), pad_xy("TP4", "1"), "USB_DN"))
for p_, v_, tp, n_ in usb:
    track([p_, (p_[0] + 0.8, p_[1]), v_], n_, 0.2)
    via(*v_, n_, d=0.6, drill=0.3)
    track([v_, tp], n_, 0.25, pcbnew.B_Cu)
gv1, gv2 = (60.4, 1.35), (60.4, 2.45)
track([c4g, gv1], "GND", 0.5)
track([u2g, gv2], "GND", 0.5)
track([gv1, gv2], "GND", 0.5)
via(*gv1, "GND", d=0.6, drill=0.3)
via(*gv2, "GND", d=0.6, drill=0.3)

# BMI088 straps. Its footprint keeps copper out from under the body, and at 0.5 mm pitch there is no
# room for a via between two pins of another net, so the straps are chosen to suit: the bottom row
# (VDDIO, CSB1, SDO1, SDO2) is all 3.3 V and joins on a bar outside the row, on to C16; on the top
# row CSB2 is left open, the ground pins either side of it (4, 6) join over it to one via, ground
# pin 2 drops to a via on its open side (pin 1 is unconnected), and VDD (3) and PS (7) run out to
# C15's 3.3 V pad, which faces them.
def u5(n):
    return pad_xy("U5", str(n))
yt = u5(3)[1]
bot3 = [u5(n) for n in (15, 14, 11, 10)]
yo_b = bot3[0][1] + 1.0
for q in bot3:
    track([q, (q[0], yo_b)], "+3V3", 0.2)
track([(bot3[0][0], yo_b), (bot3[-1][0], yo_b)], "+3V3", 0.2)
c16 = pad_xy("C16", "1")
track([(c16[0], yo_b), c16], "+3V3", 0.25)
g4, g6, g2 = u5(4), u5(6), u5(2)
ybar = yt - 0.8
track([g4, (g4[0], ybar)], "GND", 0.2)
track([g6, (g6[0], ybar)], "GND", 0.2)
track([(g4[0], ybar), (g6[0], ybar)], "GND", 0.2)
vg = ((g4[0] + g6[0]) / 2, ybar - 0.75)
track([((g4[0] + g6[0]) / 2, ybar), vg], "GND", 0.2)
via(*vg, "GND", d=0.6, drill=0.3)
v2 = (g2[0] - 0.3, yt - 1.15)
track([g2, (g2[0], yt - 0.45), v2], "GND", 0.2)
via(*v2, "GND", d=0.6, drill=0.3)
c15 = pad_xy("C15", "1")
vdd, ps = u5(3), u5(7)
track([vdd, (vdd[0], c15[1] + 0.8), c15], "+3V3", 0.2)
track([ps, (ps[0], yt - 1.0), (c15[0] + 0.3, c15[1] + 0.6), c15], "+3V3", 0.2)

# ------------------------------------------------------------------ rule areas
def no_tracks(name, x0, y0, x1, y1, layer=pcbnew.B_Cu):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True); z.SetLayer(layer)
    z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(False)
    z.SetDoNotAllowZoneFills(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
    z.SetZoneName(name)
    o = z.Outline(); o.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        o.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))
    board.Add(z)
# the bottom plane under the camera pairs stays whole, except a gap under their eastward runs at
# x 51.5 to 56, where the camera's control lines and 3.3 V cross to J8
no_tracks("CSI plane: west from J10", 3.4, 26.2, 8.2, 30.2)
no_tracks("CSI plane: south run", 3.4, 26.2, 7.4, 48.8)
no_tracks("CSI plane: east run, west part", 3.4, 43.8, 51.5, 48.8)
no_tracks("CSI plane: east run, east part", 56.0, 43.8, 69.6, 48.8)
no_tracks("CSI plane: into J8", 61.4, 43.8, 69.6, 54.8)
no_tracks("buck plane", 51.5, 0.3, 74.0, 13.0)

board.GetDesignSettings().m_TentViasFront = True
board.GetDesignSettings().m_TentViasBack = True

# Cut-line keep-outs: a top-layer strip across each cuttable link's gap, 1.5 mm past the link either
# side, so no other copper, pour included, lies where a knife cutting the link could slip (16/10/2026).
def cut_line_keepout(ref):
    fp = board.FindFootprintByReference(ref)
    p1, p2 = fp.FindPadByNumber("1"), fp.FindPadByNumber("2")
    a, c = p1.GetPosition(), p2.GetPosition()
    along_x = abs(a.x - c.x) > abs(a.y - c.y)
    cx, cy = (a.x + c.x) // 2, (a.y + c.y) // 2
    b1, b2 = p1.GetBoundingBox(), p2.GetBoundingBox()
    if along_x:
        gap, across = abs(a.x - c.x) - (b1.GetWidth() + b2.GetWidth()) // 2, max(b1.GetHeight(), b2.GetHeight())
    else:
        gap, across = abs(a.y - c.y) - (b1.GetHeight() + b2.GetHeight()) // 2, max(b1.GetWidth(), b2.GetWidth())
    ha, hc = max(gap // 2, mm(0.15)) + mm(0.1), across // 2 + mm(1.5)
    hx, hy = (ha, hc) if along_x else (hc, ha)
    z = pcbnew.ZONE(board); z.SetIsRuleArea(True); z.SetLayer(pcbnew.F_Cu); z.SetZoneName(f"cut line {ref}")
    z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(True); z.SetDoNotAllowPads(False)
    z.SetDoNotAllowZoneFills(True); z.SetDoNotAllowFootprints(False)   # no pour slivers in the gap either
    o = z.Outline(); o.NewOutline()
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        o.Append(cx + dx * hx, cy + dy * hy)
    board.Add(z)
for ref in ("JP1", "JP3", "JP4", "JP5"):
    cut_line_keepout(ref)

# ------------------------------------------------------------------ autoroute
if args.hand_only:
    pcbnew.SaveBoard(str(PCB), board)
    raise SystemExit("saved hand-drawn copper only")
dsn, ses = Path("/tmp/head.dsn"), Path("/tmp/head.ses")
for f in (dsn, ses):
    f.unlink(missing_ok=True)
if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
    raise SystemExit("DSN export failed")
best = None
for attempt in range(args.tries):
    out = Path(f"/tmp/head.{attempt}.ses")
    out.unlink(missing_ok=True)
    cmd = [args.java, "-Djava.awt.headless=true", "-jar", args.freerouting,
           "-de", str(dsn), "-do", str(out), "-mp", str(args.passes), "--gui.enabled=false"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
    m = re.findall(r"final score: [\d.]+ \((\d+) unrouted and (\d+) violations?\)", r.stdout + r.stderr)
    if not out.exists() or not m:
        print(f"try {attempt}: no result"); continue
    unrouted, viol = map(int, m[-1])
    print(f"try {attempt}: {unrouted} unrouted, {viol} violations", flush=True)
    if best is None or (unrouted, viol) < best[0]:
        best = ((unrouted, viol), out)
    if (unrouted, viol) == (0, 0):
        break
if best is None:
    raise SystemExit("Freerouting produced no session file")
shutil.copy(best[1], ses)
if not pcbnew.ImportSpecctraSES(board, str(ses)):
    raise SystemExit("SES import failed")
drop_duplicate_vias(board)
board.GetDesignSettings().m_TrackMinWidth = mm(0.12)
board.GetDesignSettings().m_CopperEdgeClearance = mm(0.25)


def drop_dangling_vias():
    """Remove vias Freerouting left with a track on one layer only (zones are not filled yet)."""
    for v in [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and not t.IsLocked()]:
        layers = set()
        for t in board.GetTracks():
            if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetCode() == v.GetNetCode() and \
                    (t.GetStart() == v.GetPosition() or t.GetEnd() == v.GetPosition()):
                layers.add(t.GetLayer())
        if len(layers) < 2 and v.GetNetname() not in ("GND",):
            board.Remove(v)
drop_dangling_vias()

def drop_dangling_tracks():
    for _ in range(5):
        board.BuildConnectivity()
        c = board.GetConnectivity()
        stubs = [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T and not t.IsLocked()
                 and c.TestTrackEndpointDangling(t, False)]
        if not stubs:
            return
        for t in stubs:
            board.Remove(t)
drop_dangling_tracks()

def fix_annular_rings(min_ring=0.15):
    for v in board.GetTracks():
        if v.Type() == pcbnew.PCB_VIA_T:
            drill = pcbnew.ToMM(v.GetDrill())
            if pcbnew.ToMM(v.GetWidth(pcbnew.F_Cu)) < drill + 2 * min_ring - 1e-6:
                v.SetWidth(mm(drill + 2 * min_ring))
fix_annular_rings()

# ------------------------------------------------------------------ pours
def zone(netname, layer, poly, priority=0, clearance=0.25, min_width=0.2, full_pads=False, locked=False):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer); z.SetNet(net(netname)); z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance)); z.SetMinThickness(mm(min_width))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if full_pads else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.25)); z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetLocked(locked)
    o = z.Outline(); o.NewOutline()
    for x, y in poly:
        o.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))
    board.Add(z)
    return z

# the module's six 5 V pads (77 to 87) and the trunk's foot, one locked copper area; pads 75 and 89
# either side stay 0.2 mm clear
zone("+5V", pcbnew.F_Cu, [(p77[0] - 0.35, p77[1] + 0.1), (p77[0] + 2.4, p77[1] + 0.1),
                          (p77[0] + 2.4, p87[1] - 0.1), (p77[0] - 0.35, p87[1] - 0.1)],
     priority=3, clearance=0.15, min_width=0.15, full_pads=True, locked=True)
# the sockets' ground pads take the pour solidly: their 0.2 mm width leaves no room for spokes
for ref in ("J9", "J10", "J8", "J11", "J12", "U2", "C1", "C2", "C4", "C17", "D1", "TP6"):   # and the buck input
    for pad in board.FindFootprintByReference(ref).Pads():
        if pad.GetNetname() == "GND":
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
whole = [(0, 0), (W, 0), (W, H), (0, H)]
zone("GND", pcbnew.B_Cu, whole, priority=0)
zone("GND", pcbnew.F_Cu, whole, priority=0, clearance=0.2)

filler = pcbnew.ZONE_FILLER(board)
filler.Fill(board.Zones())

# stitching vias on a 2 mm grid wherever both pours have room, plus a row each side of the camera
# pairs so their coplanar ground is tied to the plane
gnd = net("GND")
rule_outlines = [z.Outline() for z in board.Zones() if z.GetIsRuleArea()]
others = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetCode() != gnd.GetNetCode()]
others += [t for t in board.GetTracks() if t.GetNetCode() != gnd.GetNetCode()]
drilled = [p for fp in board.GetFootprints() for p in fp.Pads() if p.HasHole()]
drilled += [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]
holes = [(3.0, 3.0), (77.0, 3.0), (3.0, 52.0), (77.0, 52.0)]
mod = board.FindFootprintByReference("H5")
added = []
def clear_of_all(v, clearance):
    for item in others:
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            if item.IsOnLayer(layer) and v.GetEffectiveShape(layer).Collide(item.GetEffectiveShape(layer), mm(clearance)):
                return False
    for o in added:
        if (o.GetPosition() - v.GetPosition()).EuclideanNorm() < mm(1.6):
            return False
    for d in drilled:
        if v.GetEffectiveShape(pcbnew.F_Cu).Collide(d.GetEffectiveShape(pcbnew.F_Cu), mm(0.4)):
            return False
    return True
def candidates():
    step = 2.0
    y = 1.4
    while y < H - 1.0:
        x = 1.4
        while x < W - 1.0:
            yield x, y
            x += step
        y += step
    for line in CSI_LINES:                # a row each side of every pair, about 1 mm out
        for o in (0.9, -0.9):
            pts = offset(line, o)
            for a, b in zip(pts, pts[1:]):
                n = max(1, int(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 / 1.6))
                for k in range(n + 1):
                    yield a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
p5 = [z.Outline() for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "+5V"]
for x, y in candidates():
    p = pt(x, y)
    if any(o.Contains(p) or o.Collide(p, mm(0.7)) for o in p5):
        continue
    if any((x - hx) ** 2 + (y - hy) ** 2 < 3.4 ** 2 for hx, hy in holes):
        continue
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(p); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(gnd)
    if clear_of_all(v, 0.25):
        board.Add(v); added.append(v)
filler.Fill(board.Zones())
gz = {z.GetLayer(): z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "GND"}
kept = 0
for v in added:
    if all(gz[l].HitTestFilledArea(l, v.GetPosition()) for l in (pcbnew.F_Cu, pcbnew.B_Cu)):
        kept += 1
    else:
        board.Remove(v)
filler.Fill(board.Zones())
print(f"stitching vias: {kept} kept of {len(added)} candidates")
pcbnew.SaveBoard(str(PCB), board)
print("saved", PCB)
