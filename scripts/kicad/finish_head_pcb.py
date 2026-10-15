#!/usr/bin/env python3
"""Second Freerouting pass over the routed head board: route what route_head_pcb.py left open.

Keeps every track already on the board (Freerouting may move unlocked ones), lets signals use
0.13 mm clearance (JLC's 2-layer minimum is 0.127 mm), then refills the pours. Runs on the KiCad
VM after route_head_pcb.py:
    python3 scripts/kicad/finish_head_pcb.py [--passes 100]
"""
import argparse, os, re, subprocess
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("--passes", type=int, default=100)
ap.add_argument("--java", default=os.path.expanduser("~/opt/jre/bin/java"))
ap.add_argument("--freerouting", default=os.path.expanduser("~/opt/freerouting/freerouting-2.5.0.jar"))
args = ap.parse_args()


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


PCB = Path("hardware/head/head.kicad_pcb")
board = pcbnew.LoadBoard(str(PCB))
ns = board.GetDesignSettings().m_NetSettings
ns.GetDefaultNetclass().SetClearance(pcbnew.FromMM(0.13))

# the pours go to Freerouting as planes; unfilled, so it routes against the copper it can move
for z in board.Zones():
    if not z.GetIsRuleArea():
        z.UnFill()
dsn, ses = Path("/tmp/head_finish.dsn"), Path("/tmp/head_finish.ses")
for f in (dsn, ses):
    f.unlink(missing_ok=True)
if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
    raise SystemExit("DSN export failed")
r = subprocess.run([args.java, "-Djava.awt.headless=true", "-jar", args.freerouting, "-de", str(dsn),
                    "-do", str(ses), "-mp", str(args.passes), "--gui.enabled=false"],
                   capture_output=True, text=True, timeout=7200)
m = re.findall(r"final score: [\d.]+ \((\d+) unrouted and (\d+) violations?\)", r.stdout + r.stderr)
print("freerouting:", m[-1] if m else "no score")
if not ses.exists() or not pcbnew.ImportSpecctraSES(board, str(ses)):
    raise SystemExit("SES import failed")
drop_duplicate_vias(board)
for t in board.GetTracks():                   # Freerouting necks to 0.1 mm at tight pads
    if t.Type() == pcbnew.PCB_TRACE_T and t.GetWidth() < pcbnew.FromMM(0.12):
        t.SetWidth(pcbnew.FromMM(0.127))
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(str(PCB), board)
print("saved", PCB)
