#!/usr/bin/env python3
"""Annotated top render of the bridge's assembled polarised parts, for checking JLC's placement preview:
a red ring on each part's pin 1 or cathode pad, a blue ring on Q1's drain tab, and a label with the
ref, the marked pin and its CPL rotation. Hand-fitted parts' 3D models are hidden, as in JLC's preview.

Runs on the KiCad VM from the repo root (needs PIL and the DejaVu fonts):
    python3 scripts/kicad/polarity_check.py hardware/bridge/bridge-polarity.png
"""
import csv, subprocess, sys
import pcbnew
from PIL import Image, ImageDraw, ImageFont

SRC, TMP, OUT = "hardware/bridge/bridge.kicad_pcb", "/tmp/pol/bridge-pol.kicad_pcb", sys.argv[1]
subprocess.run(["mkdir", "-p", "/tmp/pol"])
W_MM, H_MM, O = 45.0, 80.0, (100.0, 50.0)
b = pcbnew.LoadBoard(SRC)
# hide the hand-fitted parts' 3D models (modules, connectors, XT30): JLC's preview won't have them
for fp in b.GetFootprints():
    if fp.IsExcludedFromBOM() or not fp.GetField("LCSC") or not fp.GetField("LCSC").GetText():
        fp.Models().clear()
pcbnew.SaveBoard(TMP, b)
subprocess.run(["cp", "hardware/bridge/bridge.kicad_pro", "/tmp/pol/bridge-pol.kicad_pro"])
subprocess.run(["kicad-cli", "pcb", "render", "--side", "top", "--quality", "high", "--background", "transparent",
                "--width", "2000", "--height", "3400", "--zoom", "0.9", "-o", "/tmp/pol/render.png", TMP],
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
img = Image.open("/tmp/pol/render.png").convert("RGBA")
bx0, by0, bx1, by1 = img.getchannel("A").point(lambda a: 255 if a > 200 else 0).getbbox()   # opaque only: the soft shadow is translucent
sx, sy = (bx1 - bx0) / W_MM, (by1 - by0) / H_MM
def px(x, y):   # board mm from the top-left corner -> image pixels
    return bx0 + x * sx, by0 + y * sy
rot = {r["Designator"]: r["Rotation"] for r in csv.DictReader(open("hardware/bridge/fab/bridge-cpl.csv"))}

# what to mark: (ref, pad, label)
MARK = [("U1", "1", "pin 1"), ("Q1", "1", "G"), ("Q1", "2", "tab D")]
MARK += [(q, "1", "G") for q in ("Q2", "Q3", "Q5")] + [("Q4", "1", "B")]
MARK += [(d, "1", "K") for d in ("D1", "D2", "D3", "D8", "D9", "D11", "D12", "D13")]

canvas = Image.new("RGBA", (img.width + 900, img.height), (255, 255, 255, 255))
canvas.alpha_composite(img, (0, 0))
dr = ImageDraw.Draw(canvas)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
RED, BLUE = (225, 20, 20, 255), (20, 70, 220, 255)
used = []
rings = []
for ref, pad, label in MARK:
    q = b.FindFootprintByReference(ref).FindPadByNumber(pad).GetPosition()
    rx, ry = px(pcbnew.ToMM(q.x) - O[0], pcbnew.ToMM(q.y) - O[1])
    used.append((rx - 24, ry - 24, rx + 24, ry + 24))     # labels keep off every ring
for ref, pad, label in MARK:
    fp = b.FindFootprintByReference(ref)
    p = fp.FindPadByNumber(pad).GetPosition()
    x, y = px(pcbnew.ToMM(p.x) - O[0], pcbnew.ToMM(p.y) - O[1])
    col = BLUE if label == "tab D" else RED
    r = 16 if label != "tab D" else 22
    dr.ellipse([x - r, y - r, x + r, y + r], outline=col, width=6)
    c = fp.GetPosition()
    cx, cy = px(pcbnew.ToMM(c.x) - O[0], pcbnew.ToMM(c.y) - O[1])
    if label != "tab D":
        txt = f"{ref} {label} {float(rot.get(ref, 0)):.0f}°"
        tw = dr.textlength(txt, font=small); th = 32
        bb = fp.GetBoundingBox(False, False) if hasattr(fp, "GetBoundingBox") else None
        hw = (pcbnew.ToMM(bb.GetWidth()) * sx / 2) if bb else 40
        hh = (pcbnew.ToMM(bb.GetHeight()) * sy / 2) if bb else 30
        cands = [(cx + hw + 10, cy - th / 2), (cx - hw - 10 - tw, cy - th / 2), (cx - tw / 2, cy - hh - th - 6),
                 (cx - tw / 2, cy + hh + 6), (cx + hw + 10, cy - th - hh), (cx - hw - 10 - tw, cy + hh),
                 (cx - tw / 2, cy - hh - 2 * th - 14), (cx - tw / 2, cy + hh + th + 14),
                 (cx + hw + 10, cy + hh + 4), (cx - hw - 10 - tw, cy - th - hh - 4)]
        for tx, ty in cands:
            box = (tx - 4, ty - 2, tx + tw + 4, ty + th + 2)
            if box[0] < 0 or box[2] > img.width: continue
            if not any(box[0] < o[2] and o[0] < box[2] and box[1] < o[3] and o[1] < box[3] for o in used): break
        used.append(box)
        dr.rectangle(box, fill=(255, 255, 255, 225), outline=col, width=2)
        dr.text((tx, ty), txt, fill=col, font=small)
        # leader from the label to the ring
        lx_, ly_ = min(max(x, box[0]), box[2]), min(max(y, box[1]), box[3])
        dr.line([lx_, ly_, x, y], fill=col, width=3)
# legend
lx = img.width + 30
lines = [("Bridge rev B: polarity check", font, (0, 0, 0, 255)), ("against JLC's placement preview", small, (0, 0, 0, 255)), ("", small, None),
         ("Red ring: pin 1 / cathode pad", small, RED), ("Blue ring: Q1's drain tab", small, BLUE), ("", small, None),
         ("Label: ref, marked pin, CPL rotation", small, (0, 0, 0, 255)), ("", small, None),
         ("K   cathode (diode / LED bar end)", small, (0, 0, 0, 255)), ("G   MOSFET gate (pin 1)", small, (0, 0, 0, 255)),
         ("B   Q4 MMBT3906 base (pin 1)", small, (0, 0, 0, 255)), ("pin 1  U1 LMR51606 (CB)", small, (0, 0, 0, 255)), ("", small, None),
         ("Check in JLC's preview that each", small, (0, 0, 0, 255)), ("part's pin-1 dot, cathode bar or", small, (0, 0, 0, 255)),
         ("tab lands on the ringed pad.", small, (0, 0, 0, 255)), ("", small, None),
         ("Q4 and Q5 matter most: wrong, and", small, (0, 0, 0, 255)), ("Q1 sits half on during shutdown.", small, (0, 0, 0, 255)), ("", small, None),
         ("Not polarised (no check): L1, F1-F6,", small, (0, 0, 0, 255)), ("all resistors and capacitors.", small, (0, 0, 0, 255)), ("", small, None),
         ("Hand-fitted parts (modules, connectors,", small, (90, 90, 90, 255)), ("XT30) are hidden, as in JLC's preview.", small, (90, 90, 90, 255))]
y = 60
for text, f, col in lines:
    if col: dr.text((lx, y), text, fill=col, font=f)
    y += 44 if f is font else 36
canvas.convert("RGB").save(OUT)
print("saved", OUT, canvas.size, "board px", (bx0, by0, bx1, by1))
