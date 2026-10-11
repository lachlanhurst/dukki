#!/usr/bin/env python3
"""Route the bridge carrier board produced by gen_bridge_pcb.py (one-shot bootstrap, like it).

1. Net classes: segment feeds 1.2 mm, head feed and buck switch node 0.6 mm, the other power
   nets 0.5 mm, signals 0.25 mm.
2. The high-current path drawn by hand and locked: XT30 + to the main fuse, the fuse to Q1's
   source, and the switched-rail trunk from Q1's tab to the branch fuses.
3. Everything else autorouted by Freerouting (Specctra DSN out, SES back in).
4. Ground pours on both layers and a switched-rail pour over the trunk, then filled.

Runs on the KiCad VM after gen_bridge_pcb.py:
    python3 scripts/kicad/route_bridge_pcb.py [--passes 40]
Freerouting and a Java 21 runtime are expected under ~/opt (see docs/bridge-pcb.md).
"""
import argparse, os, subprocess
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("--passes", type=int, default=40)
ap.add_argument("--tries", type=int, default=4, help="Freerouting runs; the one with fewest unrouted is kept")
ap.add_argument("--java", default=os.path.expanduser("~/opt/jre/bin/java"))
ap.add_argument("--freerouting", default=os.path.expanduser("~/opt/freerouting/freerouting-2.5.0.jar"))
args = ap.parse_args()

PCB = Path("hardware/bridge/bridge.kicad_pcb")
ORIGIN = (100.0, 50.0)
W, H = 45.0, 80.0
board = pcbnew.LoadBoard(str(PCB))

def mm(v):
    return pcbnew.FromMM(v)

def pt(x, y):
    return pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))

def pad_xy(ref, num):
    p = board.FindFootprintByReference(ref).FindPadByNumber(num).GetPosition()
    return pcbnew.ToMM(p.x) - ORIGIN[0], pcbnew.ToMM(p.y) - ORIGIN[1]

def full(name):
    """Schematic-local nets carry the root sheet prefix ("/VBAT_SW"); power-symbol nets do not."""
    return name if board.FindNet(name) else "/" + name

def net(name):
    n = board.FindNet(full(name))
    if n is None:
        raise SystemExit(f"no net {name!r}")
    return n

# ------------------------------------------------------------------ net classes
ns = board.GetDesignSettings().m_NetSettings
default = ns.GetDefaultNetclass()
default.SetClearance(mm(0.2)); default.SetTrackWidth(mm(0.25)); default.SetViaDiameter(mm(0.6)); default.SetViaDrill(mm(0.3))
def netclass(name, width, clearance, via=(0.8, 0.4), members=()):
    nc = pcbnew.NETCLASS(name)
    nc.SetTrackWidth(mm(width)); nc.SetClearance(mm(clearance))
    nc.SetViaDiameter(mm(via[0])); nc.SetViaDrill(mm(via[1]))
    ns.SetNetclass(name, nc)
    for m in members:
        ns.SetNetclassPatternAssignment(full(m), name)
netclass("Seg", 1.2, 0.25, members=["VSEG_A", "VSEG_B", "VSEG_C", "VSEG_D"])
netclass("Feed", 0.6, 0.2, members=["VHEAD", "BUCK_SW"])
netclass("Power", 0.5, 0.2, members=["VBAT_IN", "VBAT_F", "VBAT_SW", "GND", "+5V", "5V_BUCK", "+3V3"])
ns.RecomputeEffectiveNetclasses() if hasattr(ns, "RecomputeEffectiveNetclasses") else None

# ------------------------------------------------------------------ hand-drawn power copper
def track(points, netname, width, layer=pcbnew.F_Cu):
    for a, b in zip(points, points[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pt(*a)); t.SetEnd(pt(*b)); t.SetWidth(mm(width)); t.SetLayer(layer)
        t.SetNet(net(netname)); t.SetLocked(True)
        board.Add(t)

xt_p = pad_xy("J1", "2")           # XT30 + (pad 2)
f1_1, f1_2 = pad_xy("F1", "1"), pad_xy("F1", "2")
q_s, q_d = pad_xy("Q1", "3"), pad_xy("Q1", "2")
d2_k = pad_xy("D2", "1")
def via(x, y, netname, d=0.8, drill=0.4):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pt(x, y)); v.SetWidth(mm(d)); v.SetDrill(mm(drill))
    v.SetNet(net(netname)); v.SetLocked(True)
    board.Add(v)

# The main path is drawn on both layers, tied with via pairs: 3 mm of 1 oz copper carries about
# 5 A for a 10 degree rise (IPC-2221, outer layer), so two layers cover the 10 A design current.
# XT30 + across and down into the main fuse (the XT30 pad is through hole, both layers)
# The fuse's pads are 2.9 mm apart: the last 1.5 mm into each necks down to 1.6 mm so the two
# nets' copper cannot meet.
yv = f1_1[1] - 1.7
vin = [xt_p, (f1_1[0] - 2.5, xt_p[1]), (f1_1[0], xt_p[1] + 2.5), (f1_1[0], yv)]
track(vin, "VBAT_IN", 3.0)
track([(f1_1[0], yv), f1_1], "VBAT_IN", 1.6)
track(vin, "VBAT_IN", 3.0, pcbnew.B_Cu)
for dx in (-0.9, 0.0, 0.9):
    via(f1_1[0] + dx, yv, "VBAT_IN")
# fuse to Q1's source, TVS cathode joined on
yf = f1_2[1] + 1.7
vf = [(f1_2[0], yf), (f1_2[0], max(yf, 20.0) + 0.01), (q_s[0], 21.1)]
track([f1_2, (f1_2[0], yf)], "VBAT_F", 1.6)
track(vf + [q_s], "VBAT_F", 3.0)
track(vf, "VBAT_F", 3.0, pcbnew.B_Cu)
for dx in (-0.9, 0.0, 0.9):
    via(f1_2[0] + dx, yf, "VBAT_F")
via(q_s[0], 21.1, "VBAT_F")
track([d2_k, (d2_k[0], 19.0), (q_s[0], 21.3)], "VBAT_F", 2.0)
# switched-rail trunk: Q1 tab down, across under the gate pin, up the branch-fuse column
fuse_pads = [pad_xy(f"F{i}", "1") for i in range(2, 7)]
fx = fuse_pads[0][0]
trunk_y = 28.8
track([(q_d[0], q_d[1] + 2.0), (q_d[0], trunk_y), (fx, trunk_y)], "VBAT_SW", 2.0)
track([(fx, fuse_pads[0][1]), (fx, fuse_pads[-1][1])], "VBAT_SW", 2.0)
# bottom-layer copy of the trunk, stopping short of the fuse column so the segment feeds can
# drop through beside it; vias along it
trunk_end = fx - 1.4
track([(q_d[0], trunk_y), (trunk_end, trunk_y)], "VBAT_SW", 3.0, pcbnew.B_Cu)
x = q_d[0]
while x <= trunk_end + 0.01:
    via(x, trunk_y, "VBAT_SW")
    x += 2.4
via(trunk_end, trunk_y, "VBAT_SW")
for ref in ("C3", "C4", "C5"):
    x, y = pad_xy(ref, "1")
    track([(x, y), (x, trunk_y)], "VBAT_SW", 1.0)
# The four segment feeds by hand: left to the autorouter they knocked one another out. Each leaves
# its fuse's output pad (x about 42.45) and takes its own lane:
#   SEG A  top layer: up between D2's pads, across the top band above the XT30, down into J2 pin 2
#   SEG B  via under F3's body, bottom layer up inside SEG D's pins, across under SEG B into J3 pin 2
#   SEG C  via under F4's body, bottom layer down past the fuses, across above the WeAct board,
#          up the left edge into J4 pin 2 (about 85 mm, so 1.5 mm: 0.1 V at 3 A)
#   SEG D  via at the right edge, bottom layer up the outer lane, into J5 pin 2 from the side
VIA_X = 41.5                   # under the fuse bodies, just inside the output pads
fp2 = {r: pad_xy(r, "2") for r in ("F2", "F3", "F4", "F5")}
jp = {r: pad_xy(r, "2") for r in ("J2", "J3", "J4", "J5")}
a, ja = fp2["F2"], jp["J2"]
track([a, (a[0], 21.2), (37.6, 21.2), (37.6, 6.2), (ja[0], 6.2), ja], "VSEG_A", 1.2)
b, jb = fp2["F3"], jp["J3"]
track([b, (VIA_X, b[1])], "VSEG_B", 1.0)    # 1 mm stub: clear of the 2 mm rail column
via(VIA_X, b[1], "VSEG_B")
track([(VIA_X, b[1]), (39.0, b[1] - 2.2), (39.0, 7.8), (jb[0], 7.8), jb], "VSEG_B", 1.2, pcbnew.B_Cu)
c, jc = fp2["F4"], jp["J4"]
track([c, (VIA_X, c[1])], "VSEG_C", 1.0)
via(VIA_X, c[1], "VSEG_C")
track([(VIA_X, c[1]), (VIA_X, 34.5), (1.4, 34.5), (1.4, jc[1]), jc], "VSEG_C", 1.5, pcbnew.B_Cu)
d, jd = fp2["F5"], jp["J5"]
track([d, (44.0, d[1])], "VSEG_D", 1.2)
via(44.0, d[1], "VSEG_D")
track([(44.0, d[1]), (44.0, jd[1]), jd], "VSEG_D", 1.2, pcbnew.B_Cu)

# battery return: wide locked ground copper from the XT30's ground pad, on both layers, filling
# the gap towards the + pad and running up and down, so no signal track can pass beside the pad
gx, gy = pad_xy("J1", "1")
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    track([(gx - 2.4, gy), (gx + 1.5, gy)], "GND", 2.5, layer)
    track([(gx, gy - 2.6), (gx, gy + 2.6)], "GND", 2.5, layer)

# ------------------------------------------------------------------ keep-outs under the flush modules
# The WeAct board and the IMU breakout sit directly on the top layer: no tracks there, only the
# ground pour and tented vias (masked both sides, set below), so no bare copper faces them.
def keepout(ref, margin=0.3):
    fp = board.FindFootprintByReference(ref)
    bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
    x0, y0 = pcbnew.ToMM(bb.GetX()) - ORIGIN[0], pcbnew.ToMM(bb.GetY()) - ORIGIN[1]
    x1, y1 = pcbnew.ToMM(bb.GetRight()) - ORIGIN[0], pcbnew.ToMM(bb.GetBottom()) - ORIGIN[1]
    x0, y0, x1, y1 = max(x0, 0) + margin, max(y0, 0) + margin, min(x1, W) - margin, min(y1, H) - margin
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True); z.SetLayer(pcbnew.F_Cu)
    z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(False)
    z.SetDoNotAllowZoneFills(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
    z.SetZoneName(f"no top copper under {ref}")
    o = z.Outline(); o.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        o.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))
    board.Add(z)
# Battery return: no tracks around the XT30's ground pad or between its pads, on either layer,
# so both ground pours meet the pad from every side. Vias stay allowed for stitching.
def no_tracks(name, x0, y0, x1, y1):
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetIsRuleArea(True); z.SetLayer(layer)
        z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(False)
        z.SetDoNotAllowZoneFills(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
        z.SetZoneName(name)
        o = z.Outline(); o.NewOutline()
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            o.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))
        board.Add(z)
# (Rule areas confused Freerouting: it left unrelated nets unrouted. Locked ground copper does the
# job instead, below.)

# Not used: the modules stand about 1 mm off the board (decided 11/10/2026), so tracks and vias
# under them are fine. Kept for a build that mounts them flush.
# keepout("U2")
# keepout("U3")
board.GetDesignSettings().m_TentViasFront = True
board.GetDesignSettings().m_TentViasBack = True

# ------------------------------------------------------------------ autoroute
dsn, ses = Path("/tmp/bridge.dsn"), Path("/tmp/bridge.ses")
for f in (dsn, ses):
    f.unlink(missing_ok=True)
if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
    raise SystemExit("DSN export failed")
import re, shutil
best = None
for attempt in range(args.tries):
    out = Path(f"/tmp/bridge.{attempt}.ses")
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
board.GetDesignSettings().m_CopperEdgeClearance = mm(0.25)   # Freerouting necks down to 0.125 mm at tight pads

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
    """Remove unlocked track stubs with a loose end (Freerouting adds a few between locked vias)."""
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
    """JLC's 2-layer minimum annular ring is 0.13 mm: grow any via pad that falls short."""
    for v in board.GetTracks():
        if v.Type() == pcbnew.PCB_VIA_T:
            drill = pcbnew.ToMM(v.GetDrill())
            if pcbnew.ToMM(v.GetWidth(pcbnew.F_Cu)) < drill + 2 * min_ring - 1e-6:
                v.SetWidth(mm(drill + 2 * min_ring))
fix_annular_rings()

# ------------------------------------------------------------------ pours
def zone(netname, layer, poly, priority=0, clearance=0.25, min_width=0.25):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer); z.SetNet(net(netname)); z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance)); z.SetMinThickness(mm(min_width))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.5))
    o = z.Outline(); o.NewOutline()
    for x, y in poly:
        o.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))
    board.Add(z)
    return z

# The WeAct board's ground header pins connect solid: with thermal spokes, signal tracks under
# the module left some of them on one spoke or an island of the bottom pour.
for pad in board.FindFootprintByReference("U2").Pads():
    if pad.GetNetname() == "GND":
        pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
# Current-carrying ground pins connect solid too: thermal spokes (0.5 mm, at most four a layer)
# would choke the battery return (about 10 A at the XT30) and the segment returns (up to 4 A).
for ref in ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8"):
    for pad in board.FindFootprintByReference(ref).Pads():
        if pad.GetNetname() == "GND":
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

whole = [(0, 0), (W, 0), (W, H), (0, H)]
zone("GND", pcbnew.B_Cu, whole, priority=0)
zone("GND", pcbnew.F_Cu, whole, priority=0)
# switched rail over the trunk and fuse column, solid to the pads
zs = zone("VBAT_SW", pcbnew.F_Cu, [(23.4, 21.4), (31.0, 21.4), (31.0, 27.6), (38.6, 27.6), (38.6, 22.0),
                                    (40.4, 22.0), (40.4, 34.8), (36.4, 34.8), (36.4, 30.2), (23.4, 30.2)], priority=2)
zs.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)

filler = pcbnew.ZONE_FILLER(board)
filler.Fill(board.Zones())

# ------------------------------------------------------------------ ground stitching
# Signal tracks cut the bottom pour into islands; a via grid ties every island to the top pour.
# Candidates skip the board edge, the corner holes and the no-via areas under the modules, and
# must clear every copper item of another net by 0.3 mm. A via is kept only if it lands in
# filled ground on both layers after a refill.
gnd = net("GND")
keep_outs = [z.Outline() for z in board.Zones() if z.GetIsRuleArea()]
others = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetCode() != gnd.GetNetCode()]
others += [t for t in board.GetTracks() if t.GetNetCode() != gnd.GetNetCode()]
drilled = [p for fp in board.GetFootprints() for p in fp.Pads() if p.HasHole()]
drilled += [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]
holes = [(3.5, 3.5), (41.5, 3.5), (3.5, 76.5), (41.5, 76.5)]
added = []
def clear_of_all(v, clearance):
    for item in others:
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            if not item.IsOnLayer(layer):
                continue
            if v.GetEffectiveShape(layer).Collide(item.GetEffectiveShape(layer), mm(clearance)):
                return False
    for o in added:
        if (o.GetPosition() - v.GetPosition()).EuclideanNorm() < mm(1.8):
            return False
    for d in drilled:   # same-net holes too: keep drills apart
        if v.GetEffectiveShape(pcbnew.F_Cu).Collide(d.GetEffectiveShape(pcbnew.F_Cu), mm(0.4)):
            return False
    return True
step = 2.2
y = 1.6
while y < H - 1.2:
    x = 1.6
    while x < W - 1.2:
        p = pt(x, y)
        if all((x - hx) ** 2 + (y - hy) ** 2 > 3.6 ** 2 for hx, hy in holes) and \
                not any(o.Contains(pcbnew.VECTOR2I(p)) for o in keep_outs):
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(p); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(gnd)
            if clear_of_all(v, 0.3):
                board.Add(v); added.append(v)
        x += step
    y += step
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
