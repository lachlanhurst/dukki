#!/usr/bin/env python3
"""Bring the routed head board up to date with its schematic without re-placing or rerouting it:
what KiCad's "Update PCB from Schematic" does for added parts and changed nets.

- A footprint in the netlist but not on the board is added, linked to its symbol, at the place
  given with --place (board mm from the top-left corner, rotation, and "back" to put it on the
  bottom side).
- Every pad's net is set from the netlist; new nets are created.

Existing footprints, tracks and zones are left as they are. Run on the KiCad VM after exporting the
netlist, then route the new nets (route_one_net.py) and tidy (tidy_head_pcb.py):
    kicad-cli sch export netlist -o /tmp/head.net hardware/head/head.kicad_sch
    python3 scripts/kicad/sync_head_pcb.py /tmp/head.net --place J13 33.32 2.6 0 back
"""
import argparse, re
from pathlib import Path
import pcbnew

ap = argparse.ArgumentParser()
ap.add_argument("netlist")
ap.add_argument("--place", nargs="+", action="append", default=[],
                metavar="REF X Y ROT [back]", help="where to put a footprint the board lacks")
ap.add_argument("--kicad-share", default="/usr/share/kicad")
args = ap.parse_args()
PCB = Path("hardware/head/head.kicad_pcb")
O = (100.0, 50.0)
FPLIBS = {"DukkiHead": Path("hardware/kicad/DukkiHead.pretty")}
mm = pcbnew.FromMM

text = Path(args.netlist).read_text()
comps = {}
for block in re.split(r"\n\t\t\(comp\b", text.split("(components", 1)[1].split("\n\t(libparts", 1)[0])[1:]:
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
for block in re.split(r"\n\t\t\(net\b", text.split("(nets", 1)[1])[1:]:
    name = re.search(r"\(name \"([^\"]+)\"\)", block).group(1)
    for ref, pin in re.findall(r"\(ref \"([^\"]+)\"\)\s*\(pin \"([^\"]+)\"\)", block):
        pad_net[(ref, pin)] = name

b = pcbnew.LoadBoard(str(PCB))
place = {p[0]: p[1:] for p in args.place}

def net(name):
    n = b.FindNet(name)
    if n is None:
        n = pcbnew.NETINFO_ITEM(b, name)
        b.Add(n)
    return n

for ref, c in sorted(comps.items()):
    if b.FindFootprintByReference(ref):
        continue
    if ref not in place:
        raise SystemExit(f"{ref} is new: give its place with --place {ref} X Y ROT [back]")
    lib, name = c["footprint"].split(":")
    fp = pcbnew.FootprintLoad(str(FPLIBS.get(lib, Path(args.kicad_share) / "footprints" / f"{lib}.pretty")), name)
    fp.SetReference(ref); fp.SetValue(c["value"])
    fp.SetFPIDAsString(c["footprint"]) if hasattr(fp, "SetFPIDAsString") else fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetPath(pcbnew.KIID_PATH(f"/{c['tstamp']}"))
    fp.SetSheetname("Root"); fp.SetSheetfile("head.kicad_sch")
    for k, v in c["fields"].items():
        if k == "Footprint":
            continue
        if fp.HasField(k):
            fp.GetField(k).SetText(v)
        else:
            fp.SetField(k, v)
        fp.GetField(k).SetVisible(False)
    fp.SetDNP(c["dnp"]); fp.SetExcludedFromBOM(c["no_bom"])
    fp.Value().SetVisible(False); fp.Value().SetLayer(pcbnew.F_Fab)
    fp.Reference().SetLayer(pcbnew.F_Fab)
    x, y, rot = (float(v) for v in place[ref][:3])
    b.Add(fp)
    fp.SetPosition(pcbnew.VECTOR2I(mm(O[0] + x), mm(O[1] + y)))
    fp.SetOrientationDegrees(rot)
    if len(place[ref]) > 3 and place[ref][3] == "back":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT if hasattr(pcbnew, "FLIP_DIRECTION_LEFT_RIGHT") else False)
    fp.SetLocked(True)
    print("added", ref, "at", x, y, "back" if len(place[ref]) > 3 else "")

changed = 0
for fp in b.GetFootprints():
    ref = fp.GetReference()
    for pad in fp.Pads():
        want = pad_net.get((ref, pad.GetNumber()))
        if want and pad.GetNetname() != want:
            pad.SetNet(net(want)); changed += 1
print("pad nets set:", changed)
pcbnew.SaveBoard(str(PCB), b)
print("saved", PCB)
