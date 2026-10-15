#!/usr/bin/env python3
"""Join the smallest stranded piece of one net to the rest of it: a maze router on a 0.1 mm grid
over both copper layers, vias allowed but costly, for the last connection an autorouter leaves.

Every other net's copper is an obstacle, grown by the clearance plus half the track (or via) width;
keep-out rule areas and board holes block too. The cheapest path from the small island to any
copper of the rest goes onto the board as locked tracks and vias; then refill and run DRC.

Runs on the KiCad VM from the repo root, on the head board (edit PCB for another):
    python3 scripts/kicad/route_one_net.py /I2C8_SCL [x0 y0 x1 y1 [via_cost]]
The window is in board mm from the top-left corner (default 38 20 76 46); a smaller one is faster.
Used 14/10/2026 for the head board's last connection (docs/head-pcb.md 9.3).
"""
import sys, heapq, pcbnew
NET = sys.argv[1]
X0, Y0, X1, Y1 = (float(v) for v in sys.argv[2:6]) if len(sys.argv) > 5 else (38, 20, 76, 46)
VIA_COST = float(sys.argv[6]) if len(sys.argv) > 6 else 40.0   # in grid steps (0.1 mm)
O = (100.0, 50.0); mm = pcbnew.FromMM; tomm = pcbnew.ToMM
PCB = "hardware/head/head.kicad_pcb"
b = pcbnew.LoadBoard(PCB)
net = b.FindNet(NET); nc = net.GetNetCode()
HOLE_CL = 0.25                        # board setup: copper to hole
W, CL, VD = 0.2, 0.2, 0.6             # track width, clearance (the largest net class), via diameter
LAY = [pcbnew.F_Cu, pcbnew.B_Cu]
STEP = 0.1
nx, ny = int((X1 - X0) / STEP) + 1, int((Y1 - Y0) / STEP) + 1
def P(i, j): return pcbnew.VECTOR2I(mm(O[0] + X0 + i * STEP), mm(O[1] + Y0 + j * STEP))

# islands of the net
b.BuildConnectivity(); con = b.GetConnectivity()
items = [t for t in b.GetTracks() if t.GetNetCode() == nc] + [p for f in b.GetFootprints() for p in f.Pads() if p.GetNetCode() == nc]
groups, seen = [], set()
for it in items:
    if it.m_Uuid.AsString() in seen: continue
    g = [x for x in con.GetConnectedItems(it) if x.GetNetCode() == nc] + [it]
    for x in g: seen.add(x.m_Uuid.AsString())
    groups.append(g)
groups.sort(key=len)
small, big = groups[0], [x for g in groups[1:] for x in g]
print("islands", [len(g) for g in groups])

NS = b.GetDesignSettings().m_NetSettings
SELF_CL = tomm(NS.GetEffectiveNetClass(NET).GetClearance())
def item_cl(o):
    """Clearance between this net and o: the larger of the two net classes' (footprint copper with
    no net, such as a solder jumper's bridge, takes 0.2 mm)."""
    try:
        other = tomm(NS.GetEffectiveNetClass(o.GetNetname()).GetClearance()) if o.GetNetCode() > 0 else 0.2
    except Exception:
        other = 0.2
    return max(SELF_CL, other)
def polyset(objs, layer, extra, own_clearance=True):
    ps = pcbnew.SHAPE_POLY_SET()
    for o in objs:
        if o.IsOnLayer(layer):
            grow = (item_cl(o) if own_clearance else 0) + extra
            o.TransformShapeToPolygon(ps, layer, mm(grow), mm(0.005), pcbnew.ERROR_OUTSIDE)
    ps.Simplify()
    return ps
others = [t for t in b.GetTracks() if t.GetNetCode() != nc] + [p for f in b.GetFootprints() for p in f.Pads() if p.GetNetCode() != nc]
others += [g for f in b.GetFootprints() for g in f.GraphicalItems()        # footprint copper: jumper bridges
           if g.IsOnLayer(pcbnew.F_Cu) or g.IsOnLayer(pcbnew.B_Cu)]
rules = [z for z in b.Zones() if z.GetIsRuleArea()]
rules += [z for f in b.GetFootprints() for z in f.Zones() if z.GetIsRuleArea()]   # e.g. the BMI088's keep-out
holes = [p for f in b.GetFootprints() for p in f.Pads() if p.HasHole() and p.GetNetCode() != nc]
block, vblock, src, dst = [], [], [], []
for L in LAY:
    blk = polyset(others, L, W / 2)
    vbl = polyset(others, L, VD / 2)
    for z in rules:                    # keep-outs, grown by half a track (a via) so copper stays out
        if z.IsOnLayer(L) and z.GetDoNotAllowTracks():
            grown = pcbnew.SHAPE_POLY_SET(z.Outline())
            grown.Inflate(mm(W / 2), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.005))
            blk.Append(grown)
    for h in holes:                    # drilled holes of every other pad, at the hole clearance
        h.TransformHoleToPolygon(blk, mm(HOLE_CL + W / 2), mm(0.005), pcbnew.ERROR_OUTSIDE)
        h.TransformHoleToPolygon(vbl, mm(HOLE_CL + VD / 2), mm(0.005), pcbnew.ERROR_OUTSIDE)
    blk.Simplify(); vbl.Simplify()
    block.append(blk); vblock.append(vbl)
    src.append(polyset(small, L, 0, False)); dst.append(polyset(big, L, 0, False))
# holes (any net, including NPTH) block both layers for tracks and vias
holes = [p for f in b.GetFootprints() for p in f.Pads() if p.HasHole() and p.GetNetCode() != nc]
edge = b.GetBoardEdgesBoundingBox()

def free(L, i, j):
    q = P(i, j)
    return not block[L].Contains(q)
def via_ok(i, j):
    q = P(i, j)
    if any(vblock[L].Contains(q) for L in (0, 1)): return False
    for z in rules:
        if z.GetDoNotAllowVias() and z.Outline().Contains(q): return False
    return True

dist = {}
pq = []
for L in (0, 1):
    for i in range(nx):
        for j in range(ny):
            if src[L].Contains(P(i, j)) and free(L, i, j):
                dist[(L, i, j)] = 0; heapq.heappush(pq, (0, L, i, j))
print("start cells", len(pq))
prev = {}
MOVES = [(1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),(1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
goal = None
fcache, vcache = {}, {}
while pq:
    d, L, i, j = heapq.heappop(pq)
    if d > dist.get((L, i, j), 1e18): continue
    if dst[L].Contains(P(i, j)):
        goal = (L, i, j); break
    nbrs = []
    for di, dj, c in MOVES:
        ii, jj = i + di, j + dj
        if 0 <= ii < nx and 0 <= jj < ny:
            k = (L, ii, jj)
            if k not in fcache: fcache[k] = free(L, ii, jj)
            if fcache[k]: nbrs.append((k, c + (0.3 if L == 1 else 0)))
    if (i, j) not in vcache: vcache[(i, j)] = via_ok(i, j)
    if vcache[(i, j)]:
        k = (1 - L, i, j)
        if k not in fcache: fcache[k] = free(1 - L, i, j)
        if fcache[k]: nbrs.append((k, VIA_COST))
    for k, c in nbrs:
        nd = d + c
        if nd < dist.get(k, 1e18):
            dist[k] = nd; prev[k] = (L, i, j); heapq.heappush(pq, (nd, *k))
if not goal:
    sys.exit("no path")
path = [goal]
while path[-1] in prev: path.append(prev[path[-1]])
path.reverse()
print("path cells", len(path), "cost", round(dist[goal], 1))
# turn the cell path into tracks and vias: break at layer changes and direction changes
added = 0
seg = [path[0]]
def flush(seg):
    global added
    pts = [seg[0]]
    for a, m, c in zip(seg, seg[1:], seg[2:]):
        if (m[1] - a[1], m[2] - a[2]) != (c[1] - m[1], c[2] - m[2]): pts.append(m)
    pts.append(seg[-1])
    for a, c in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b); t.SetStart(P(a[1], a[2])); t.SetEnd(P(c[1], c[2])); t.SetWidth(mm(W))
        t.SetLayer(LAY[a[0]]); t.SetNet(net); t.SetLocked(True); b.Add(t); added += 1
for a, c in zip(path, path[1:]):
    if a[0] != c[0]:
        if len(seg) > 1: flush(seg)
        v = pcbnew.PCB_VIA(b); v.SetPosition(P(a[1], a[2])); v.SetWidth(mm(VD)); v.SetDrill(mm(0.3)); v.SetNet(net); v.SetLocked(True); b.Add(v)
        seg = [c]
    else:
        seg.append(c)
if len(seg) > 1: flush(seg)
print("tracks added", added, "vias", sum(1 for a, c in zip(path, path[1:]) if a[0] != c[0]))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(PCB, b)
