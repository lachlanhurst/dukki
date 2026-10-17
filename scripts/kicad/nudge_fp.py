#!/usr/bin/env python3
"""Move footprints on a routed board by a small step, taking the track ends on their pads (and
the cut-line keep-out of a jumper) with them, so nothing needs rerouting:
    python3 scripts/kicad/nudge_fp.py hardware/head_ext/head_ext.kicad_pcb JP3 0 -0.3 [C11 0 -0.2 ...]
Check with DRC afterwards: the moved ends turn their segments slightly.
"""
import sys
import pcbnew

PCB = sys.argv[1]
b = pcbnew.LoadBoard(PCB)
mm = pcbnew.FromMM
args = sys.argv[2:]
for ref, dx, dy in zip(args[0::3], args[1::3], args[2::3]):
    d = pcbnew.VECTOR2I(mm(float(dx)), mm(float(dy)))
    fp = b.FindFootprintByReference(ref)
    pads = list(fp.Pads())
    moved = 0
    for t in b.GetTracks():
        for p in pads:
            if p.GetNetCode() != t.GetNetCode():
                continue
            if t.Type() == pcbnew.PCB_VIA_T:
                continue
            for get, put in ((t.GetStart, t.SetStart), (t.GetEnd, t.SetEnd)):
                e = get()
                if any(p.IsOnLayer(l) and l == t.GetLayer() for l in (pcbnew.F_Cu, pcbnew.B_Cu)) and \
                        p.HitTest(e):
                    put(e + d); moved += 1
    fp.Move(d)
    for z in b.Zones():
        if z.GetIsRuleArea() and z.GetZoneName() == f"cut line {ref}":
            z.Move(d)
    print(ref, "moved", dx, dy, "with", moved, "track ends")
pcbnew.SaveBoard(PCB, b)
