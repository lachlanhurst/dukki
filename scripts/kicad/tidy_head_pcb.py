#!/usr/bin/env python3
"""Tidy the routed head board: remove the ground stitching, straighten and widen the autorouted
tracks, then stitch again sparingly.

1. Unlocked ground vias with no track on them (the stitching grid) are removed.
1b. Unlocked ground tracks are removed: the pours carry ground.
2. Wherever exactly two unlocked segments of one net meet on one layer, with no pad or via at the
   joint, they are replaced by one straight segment if it clears every other net's copper by the
   larger of the two net classes' clearances and stays out of the keep-outs. Repeated until nothing
   changes: this takes out Freerouting's small jogs and the maze router's staircase.
3. Every unlocked segment narrower than its net class is widened to it where it fits.
4. Ground stitching again: a 3.5 mm grid, plus rows beside the camera pairs, only where both ground
   pours are filled; then the pours are refilled.

Runs on the KiCad VM after route_head_pcb.py and finish_head_pcb.py (and route_one_net.py), one
stage per process (pcbnew's Python crashes when a long edit session reuses wrappers of tracks it
added), the straightening repeated until it reports 0:
    python3 scripts/kicad/tidy_head_pcb.py unstitch
    python3 scripts/kicad/tidy_head_pcb.py unground
    python3 scripts/kicad/tidy_head_pcb.py straighten      # until "0 joints straightened"
    python3 scripts/kicad/tidy_head_pcb.py widen
    python3 scripts/kicad/tidy_head_pcb.py stitch
"""
import math, sys
from pathlib import Path
import pcbnew

PCB = Path("hardware/head/head.kicad_pcb")
O = (100.0, 50.0)
W, H = 80.0, 55.0
mm, tomm = pcbnew.FromMM, pcbnew.ToMM
STAGE = sys.argv[1]
b = pcbnew.LoadBoard(str(PCB))
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)

def pt(x, y):
    return pcbnew.VECTOR2I(mm(O[0] + x), mm(O[1] + y))

# ------------------------------------------------------------------ 1. stitching off
if STAGE == "unstitch":
    tracks = [t for t in b.GetTracks()]
    ends = {}
    for t in tracks:
        if t.Type() == pcbnew.PCB_TRACE_T:
            for p in (t.GetStart(), t.GetEnd()):
                ends.setdefault((p.x, p.y), []).append(t)
    removed = 0
    for v in [t for t in tracks if t.Type() == pcbnew.PCB_VIA_T and not t.IsLocked() and t.GetNetname() == "GND"]:
        if not ends.get((v.GetPosition().x, v.GetPosition().y)):
            b.Remove(v); removed += 1
    print("stitching vias removed:", removed)
    for z in b.Zones():
        if not z.GetIsRuleArea():
            z.UnFill()

# ------------------------------------------------------------------ 1b. ground tracks off
# Freerouting routes GND as tracks before any pour exists; on the finished board they only duplicate
# the pours and show as copper across the ground pads' thermal-relief gaps. All unlocked GND tracks
# go (the hand-drawn ones are locked and stay); the pours then reach every ground pad, checked by
# DRC. C9's ground pad gets a solid connection: with its track gone too few spokes reached the pour.
if STAGE == "unground":
    gt = [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == "GND" and not t.IsLocked()]
    for t in gt:
        b.Remove(t)
    print("ground tracks removed:", len(gt))
    for ref, num in (("C9", "2"),):
        b.FindFootprintByReference(ref).FindPadByNumber(num).SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

# ------------------------------------------------------------------ obstacle checks
rules = [z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowTracks()]
pads = [p for fp in b.GetFootprints() for p in fp.Pads()]
holes = [p for p in pads if p.HasHole()]

# net classes by net name, from the board's own rules (a track not yet on the board reports the
# default class, so its own lookup cannot be trusted)
NS = b.GetDesignSettings().m_NetSettings
def netclass(item):
    return NS.GetEffectiveNetClass(item.GetNetname())

def clearance_of(item):
    return netclass(item).GetClearance()

def width_of_class(item):
    return netclass(item).GetTrackWidth()

EDGE = mm(0.3)
def fits(seg, ignore):
    """seg: a PCB_TRACK, on the board or not; ignore: UUIDs of items to leave out (the ones it replaces). True if it clears every other net's copper on its layer,
    the holes, the keep-outs and the board edge."""
    layer, net = seg.GetLayer(), seg.GetNetCode()
    shape = seg.GetEffectiveShape(layer)
    half = seg.GetWidth() // 2
    for p in (seg.GetStart(), seg.GetEnd()):
        x, y = tomm(p.x) - O[0], tomm(p.y) - O[1]
        if min(x, y, W - x, H - y) < tomm(EDGE + half):
            return False
    bb = seg.GetBoundingBox()
    bb.Inflate(mm(0.8))
    cl_self = clearance_of(seg)
    for it in candidates(layer):
        if it.GetNetCode() == net or it.m_Uuid.AsString() in ignore:
            continue
        if not bb.Intersects(it.GetBoundingBox()):
            continue
        cl = max(cl_self, clearance_of(it))
        if it.GetEffectiveShape(layer).Collide(shape, cl - 1):
            return False
    for h in holes:
        if h.GetNetCode() != net and bb.Intersects(h.GetBoundingBox()):
            if h.GetEffectiveHoleShape().Collide(shape, mm(0.25)):
                return False
    for z in rules:
        if z.IsOnLayer(layer) and z.Outline().Collide(pcbnew.SEG(seg.GetStart(), seg.GetEnd()), half):
            return False
    return True

def candidates(layer):
    out = [p for p in pads if p.IsOnLayer(layer)]
    out += [t for t in b.GetTracks() if t.IsOnLayer(layer)]
    return out

def temp_track(a, c, layer, net, width):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(a); t.SetEnd(c); t.SetLayer(layer); t.SetNet(net); t.SetWidth(width)
    return t

def anchored(p, layer, net):
    """A pad or via of this net at p: the joint is a real connection, not a free bend."""
    for v in b.GetTracks():
        if v.Type() == pcbnew.PCB_VIA_T and v.GetPosition() == p:
            return True
    for pd in pads:
        if pd.IsOnLayer(layer) and pd.HitTest(p):
            return True
    return False

# ------------------------------------------------------------------ 2. straighten
merged = 0
for sweep in range(1 if STAGE == "straighten" else 0):
    changed = 0
    joints = {}
    for t in b.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T:
            for p in (t.GetStart(), t.GetEnd()):
                joints.setdefault((p.x, p.y, t.GetLayer()), []).append(t)
    done = set()
    for (x, y, layer), ts in joints.items():
        if len(ts) != 2:
            continue
        t1, t2 = ts
        if t1.IsLocked() or t2.IsLocked() or t1.GetNetCode() != t2.GetNetCode():
            continue
        if id(t1) in done or id(t2) in done:
            continue
        p = pcbnew.VECTOR2I(x, y)
        if anchored(p, layer, t1.GetNetCode()):
            continue
        a = t1.GetEnd() if t1.GetStart() == p else t1.GetStart()
        c = t2.GetEnd() if t2.GetStart() == p else t2.GetStart()
        if a == c:
            continue
        width = max(t1.GetWidth(), t2.GetWidth(), width_of_class(t1))
        new = temp_track(a, c, layer, t1.GetNet(), width)
        if not fits(new, {t1.m_Uuid.AsString(), t2.m_Uuid.AsString()}):
            width = max(t1.GetWidth(), t2.GetWidth())
            new = temp_track(a, c, layer, t1.GetNet(), width)
            if not fits(new, {t1.m_Uuid.AsString(), t2.m_Uuid.AsString()}):
                continue
        b.Remove(t1); b.Remove(t2); b.Add(new); new.thisown = 0
        done.update((id(t1), id(t2)))
        changed += 1
    merged += changed
    print(f"sweep {sweep}: {changed} joints straightened", flush=True)
    if not changed:
        break

# ------------------------------------------------------------------ 3. widen
widened = narrow = 0
for t in [t for t in b.GetTracks() if STAGE == "widen" and t.Type() == pcbnew.PCB_TRACE_T and not t.IsLocked()]:
    want = width_of_class(t)
    if t.GetWidth() >= want:
        continue
    narrow += 1
    old = t.GetWidth()
    t.SetWidth(want)
    if fits(t, {t.m_Uuid.AsString()}):
        widened += 1
    else:
        t.SetWidth(old)
if STAGE == "widen":
    print(f"widened {widened} of {narrow} narrow segments")

# ------------------------------------------------------------------ 4. sparse stitching
filler = pcbnew.ZONE_FILLER(b)
if STAGE != "stitch":
    for z in b.Zones():
        if not z.GetIsRuleArea():
            z.UnFill()
    pcbnew.SaveBoard(str(PCB), b)
    print("saved", PCB, "after", STAGE)
    sys.exit(0)
filler.Fill(b.Zones())
gnd = b.FindNet("GND")
others = [p for p in pads if p.GetNetCode() != gnd.GetNetCode()]
others += [t for t in b.GetTracks() if t.GetNetCode() != gnd.GetNetCode()]
drilled = [p for p in holes] + [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]
p5 = [z.Outline() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "+5V"]
corner_holes = [(3.0, 3.0), (77.0, 3.0), (3.0, 52.0), (77.0, 52.0)]
added = []
def place(x, y, spacing):
    q = pt(x, y)
    if any((x - hx) ** 2 + (y - hy) ** 2 < 3.4 ** 2 for hx, hy in corner_holes):
        return
    if any(o.Contains(q) for o in p5):
        return
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(q); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(gnd)
    for o in added:
        if (o.GetPosition() - q).EuclideanNorm() < mm(spacing):
            return
    for d in drilled:
        if v.GetEffectiveShape(pcbnew.F_Cu).Collide(d.GetEffectiveShape(pcbnew.F_Cu), mm(0.4)):
            return
    for it in others:
        for L in LAYERS:
            if it.IsOnLayer(L) and v.GetEffectiveShape(L).Collide(it.GetEffectiveShape(L), mm(0.25)):
                return
    b.Add(v); v.thisown = 0; added.append(v)
# rows beside the camera pairs first: the pairs' coplanar ground, about 1 mm out, every 2.5 mm
csi = [t for t in b.GetTracks() if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname().startswith("/CSI_")]
for t in csi:
    a, c = t.GetStart(), t.GetEnd()
    ax, ay, cx, cy = tomm(a.x) - O[0], tomm(a.y) - O[1], tomm(c.x) - O[0], tomm(c.y) - O[1]
    n = max(1, int(math.hypot(cx - ax, cy - ay) / 2.5))
    dx, dy = (cx - ax), (cy - ay)
    ln = math.hypot(dx, dy) or 1
    nx_, ny_ = -dy / ln, dx / ln
    for k in range(n + 1):
        x, y = ax + dx * k / n, ay + dy * k / n
        for s in (0.9, -0.9):
            place(x + nx_ * s, y + ny_ * s, 2.0)
y = 1.75
while y < H - 1.0:
    x = 1.75
    while x < W - 1.0:
        place(x, y, 3.0)
        x += 3.5
    y += 3.5
filler.Fill(b.Zones())
gz = {z.GetLayer(): z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "GND"}
kept = 0
for v in added:
    if all(gz[L].HitTestFilledArea(L, v.GetPosition()) for L in LAYERS):
        kept += 1
    else:
        b.Remove(v)
filler.Fill(b.Zones())
print(f"straightened {merged} joints; stitching vias: {kept}")
pcbnew.SaveBoard(str(PCB), b)
print("saved", PCB)
