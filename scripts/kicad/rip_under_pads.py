#!/usr/bin/env python3
"""Take out the tracks and vias of other nets that the given footprints' pads land on, or come
closer to than the clearance: after sync_head_pcb.py puts new parts on a routed board. The nets
they belonged to are then joined again with route_one_net.py. Runs on the KiCad VM, a process of
its own (pcbnew crashes when a board is edited further after removals):
    python3 scripts/kicad/rip_under_pads.py hardware/head_ext/head_ext.kicad_pcb U5 J3 J11 J12
"""
import sys
import pcbnew

PCB, REFS = sys.argv[1], sys.argv[2:]
b = pcbnew.LoadBoard(PCB)
CL = pcbnew.FromMM(0.2 + 0.02)
pads = [p for r in REFS for p in b.FindFootprintByReference(r).Pads()]
gone, nets = {}, set()
for t in b.GetTracks():
    for p in pads:
        if p.GetNetCode() == t.GetNetCode() and p.GetNetCode() > 0:
            continue
        hit = False
        for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
            if p.IsOnLayer(lay) and t.IsOnLayer(lay) and \
                    p.GetEffectiveShape(lay).Collide(t.GetEffectiveShape(lay), CL):
                hit = True
        if hit:
            gone[t.m_Uuid.AsString()] = t; nets.add(t.GetNetname())
            break
for t in gone.values():
    b.Remove(t)
pcbnew.SaveBoard(PCB, b)
print("removed", len(gone), "tracks and vias of", sorted(nets))
