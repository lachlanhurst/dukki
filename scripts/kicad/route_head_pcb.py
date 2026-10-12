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
netclass("Power", 0.6, 0.2, members=["VBAT_IN", "+5V", "GND"])
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
# the unbroken bottom plane, ground pour either side. Each pair leaves J10's outer row eastwards,
# turns south down the corridor beside the module, east below the BMI088, and south into J8. The
# turns go right, left, right, so N (north of P leaving J10) arrives east of P, as J8's pinout has
# it, and the pairs nest without crossing: D0 outermost, C innermost.
#   (pair, socket pin N, J8 pin N, south-run x, east-run y, final south-run x)
PAIRS = [("D0", "128", "2", 50.2, 44.6, 68.5),
         ("D1", "134", "5", 48.6, 46.2, 65.5),
         ("C", "140", "8", 47.0, 47.8, 62.5)]
for name, sp, jp, xs, ye, xf in PAIRS:
    sn = pad_xy("J10", sp)                    # N pad; P is the next even pin, 0.4 mm below
    sp_ = (sn[0], sn[1] + 0.4)
    jn, jpp = pad_xy("J8", jp), pad_xy("J8", str(int(jp) + 1))
    jog = 51.6                                # leave the pair's pitch, fan out to J8's 1 mm pitch
    n_pts = [sn, (xs + 0.2, sn[1]), (xs + 0.2, ye - 0.2), (xf + 0.2, ye - 0.2), (xf + 0.2, jog),
             (jn[0], jog + 0.8), jn]
    p_pts = [sp_, (xs - 0.2, sp_[1]), (xs - 0.2, ye + 0.2), (xf - 0.2, ye + 0.2), (xf - 0.2, jog),
             (jpp[0], jog + 0.8), jpp]
    track(n_pts, f"CSI_{name}_N", 0.2)
    track(p_pts, f"CSI_{name}_P", 0.2)

# 5 V trunk: L1's output pad and the three output capacitors joined on the top; from there down
# to the bottom layer and straight across under the module, 2.5 mm wide, to a row of vias beside
# the module's 5 V pins (77 to 87, J9's outer row), which a small locked copper area on the top
# joins (below). The top layer left of J9 stays free for the outer row's signals to escape.
l2 = pad_xy("L1", "2")
cs = [pad_xy(r, "1") for r in ("C6", "C7", "C8")]
yb = cs[0][1]
track([l2, (cs[0][0], yb)], "+5V", 1.5)
track([(cs[0][0], yb), (cs[-1][0], yb)], "+5V", 1.5)
p77, p87 = pad_xy("J9", "77"), pad_xy("J9", "87")
XV = p77[0] - 1.25                            # the via column, left of the outer row
va = (cs[0][0] - 2.0, yb)                     # the drop to the bottom layer, left of C6
track([(cs[0][0], yb), va], "+5V", 1.5)
for dy in (-0.9, 0.0, 0.9):
    via(va[0], va[1] + dy, "+5V")
vy = [p77[1] + 0.2, (p77[1] + p87[1]) / 2, p87[1] - 0.2]
track([va, (XV, vy[1])], "+5V", 2.5, pcbnew.B_Cu)
track([(XV, vy[0]), (XV, vy[-1])], "+5V", 2.0, pcbnew.B_Cu)
for y in vy:
    via(XV, y, "+5V", d=0.7, drill=0.35)
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
# the bottom plane under the camera pairs stays whole
no_tracks("CSI plane: east from J10", 44.0, 24.6, 51.2, 29.0)
no_tracks("CSI plane: south run", 46.2, 24.6, 51.2, 49.0)
no_tracks("CSI plane: east run", 46.2, 43.6, 69.6, 49.0)
no_tracks("CSI plane: into J8", 61.4, 43.6, 69.6, 54.8)

board.GetDesignSettings().m_TentViasFront = True
board.GetDesignSettings().m_TentViasBack = True

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
    m = re.findall(r"final score: [\d.]+ \((\d+) unrouted and (\d+) violations\)", r.stdout + r.stderr)
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
# above and below stay 0.2 mm clear
zone("+5V", pcbnew.F_Cu, [(XV - 0.6, p77[1] - 0.1), (p77[0] + 0.35, p77[1] - 0.1),
                          (p77[0] + 0.35, p87[1] + 0.1), (XV - 0.6, p87[1] + 0.1)],
     priority=3, clearance=0.15, min_width=0.15, full_pads=True, locked=True)
# the sockets' ground pads take the pour solidly: their 0.2 mm width leaves no room for spokes
for ref in ("J9", "J10"):
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
    for _, sp, jp, xs, ye, xf in PAIRS:   # beside each pair's runs
        for yy in [y / 2 for y in range(52, 96, 3)]:
            yield xs + 0.9, yy
            yield xs - 0.9, yy
        for xx in [x / 2 for x in range(96, 140, 3)]:
            yield xx, ye - 0.9
            yield xx, ye + 0.9
for x, y in candidates():
    p = pt(x, y)
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
