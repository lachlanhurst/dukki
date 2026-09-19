#!/usr/bin/env python3
"""Generate the KiCad 8 footprint and symbol for the WeAct STM32G474 core board (LQFP48, V1.0).

Dimensions are from WeAct's board outline drawing
(docs/datasheets/WeAct-STM32G47xCxTxCoreBoard_V10 Board Shape.pdf): 36.28 x 28.14 mm,
two 2 x 12 headers at 2.54 mm pitch, outer rows 1.37 mm from the long edges, first column
6.97 mm from the USB-C edge. Pin names follow the schematic connectors P1 (pads 1 to 24) and
P2 (pads 25 to 48); odd pads are the inner row, even pads the outer row, as on the silkscreen.

Run from the repo root:  python3 scripts/kicad/gen_weact_kicad.py
"""
import uuid
from pathlib import Path

NAME = "WeAct_STM32G474CoreBoard_LQFP48"
LIB = "WeAct_STM32G474CoreBoard"
OUT = Path("hardware/kicad")

W, H = 36.28, 28.14           # board outline, mm
R = 2.0                       # corner radius, estimated from the drawing
PITCH = 2.54
X0 = 6.97 - W / 2             # first column centre, board centre as origin
Y_OUT, Y_IN = H / 2 - 1.37, H / 2 - 1.37 - PITCH   # outer and inner row distance from centre
PAD, DRILL = 1.7, 1.0         # KiCad PinHeader_P2.54mm convention

# P1 (bottom edge on the drawing, y > 0), schematic pin order 1..24
P1 = ["VCC", "VCC", "GND", "GND", "PB12", "PB13", "PB14", "PB15", "NC", "PA8", "PA9", "PA10",
      "PA11", "PA12", "PA15", "NC", "NC", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8/BOOT0", "PB9"]
# P2 (top edge, y < 0), schematic pin order 1..24, footprint pads 25..48
P2 = ["3V3", "3V3", "GND", "GND", "PB10", "PB11", "PB2", "VREF+", "PB0", "PB1", "PA7", "NC",
      "PA5", "PA6", "PA3", "PA4", "PA1", "PA2", "NRST", "PA0", "PC14", "PC15", "VB", "PC13"]
assert len(P1) == len(P2) == 24

def u():
    return f'(uuid "{uuid.uuid4()}")'

def fmt(v):
    return f"{v:.3f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)

# ---------------------------------------------------------------- footprint
def line(a, b, layer, width):
    return (f"  (fp_line (start {fmt(a[0])} {fmt(a[1])}) (end {fmt(b[0])} {fmt(b[1])}) "
            f"(stroke (width {width}) (type solid)) (layer \"{layer}\") {u()})")

def arc(start, mid, end, layer, width):
    return (f"  (fp_arc (start {fmt(start[0])} {fmt(start[1])}) (mid {fmt(mid[0])} {fmt(mid[1])}) "
            f"(end {fmt(end[0])} {fmt(end[1])}) (stroke (width {width}) (type solid)) (layer \"{layer}\") {u()})")

def rounded_rect(w, h, r, layer, width):
    x, y = w / 2, h / 2
    k = r * (1 - 0.70710678)   # offset of the 45 degree arc midpoint from the corner
    out = [
        line((-x + r, -y), (x - r, -y), layer, width),
        line((x, -y + r), (x, y - r), layer, width),
        line((x - r, y), (-x + r, y), layer, width),
        line((-x, y - r), (-x, -y + r), layer, width),
        # arcs drawn clockwise in KiCad's y-down frame
        arc((-x, -y + r), (-x + k, -y + k), (-x + r, -y), layer, width),
        arc((x - r, -y), (x - k, -y + k), (x, -y + r), layer, width),
        arc((x, y - r), (x - k, y - k), (x - r, y), layer, width),
        arc((-x + r, y), (-x + k, y - k), (-x, y - r), layer, width),
    ]
    return out

def rect(x1, y1, x2, y2, layer, width):
    return [line((x1, y1), (x2, y1), layer, width), line((x2, y1), (x2, y2), layer, width),
            line((x2, y2), (x1, y2), layer, width), line((x1, y2), (x1, y1), layer, width)]

def text(s, x, y, layer, size=1.0, rot=0):
    return (f"  (fp_text user \"{s}\" (at {fmt(x)} {fmt(y)} {rot}) (layer \"{layer}\") {u()} "
            f"(effects (font (size {size} {size}) (thickness 0.15))))")

def prop(name, value, x, y, layer, hide=False):
    h = " (hide yes)" if hide else ""
    return (f"  (property \"{name}\" \"{value}\" (at {fmt(x)} {fmt(y)} 0) (layer \"{layer}\"){h} {u()} "
            f"(effects (font (size 1 1) (thickness 0.15))))")

def pad(num, name, x, y, shape="circle"):
    return (f"  (pad \"{num}\" thru_hole {shape} (at {fmt(x)} {fmt(y)}) (size {PAD} {PAD}) (drill {DRILL}) "
            f"(layers \"*.Cu\" \"*.Mask\") (remove_unused_layers no) (pinfunction \"{name}\") {u()})")

fp = [f"(footprint \"{NAME}\"", "  (version 20240108)", "  (generator \"pcbnew\")",
      "  (generator_version \"8.0\")", "  (layer \"F.Cu\")",
      "  (descr \"WeAct Studio STM32G474 core board V1.0, LQFP48 (CBT6/CET6) variant. 36.28 x 28.14 mm, "
      "two 2x12 2.54 mm headers, USB-C on the left edge, 4-pin SWD header at the right edge. "
      "Pads 1-24 are schematic connector P1 (VCC side), 25-48 are P2 (3V3 side); odd pads inner row, even pads outer row.\")",
      "  (tags \"WeAct STM32G474 core board module header\")",
      prop("Reference", "REF**", 0, -(H / 2 + 1.2), "F.SilkS"),
      prop("Value", NAME, 0, H / 2 + 1.2, "F.Fab"),
      prop("Footprint", "", 0, 0, "F.Fab", hide=True),
      prop("Datasheet", "https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard", 0, 0, "F.Fab", hide=True),
      prop("Description", "", 0, 0, "F.Fab", hide=True),
      "  (attr through_hole)"]

# Fab: board outline, USB-C receptacle (protrudes about 1 mm past the left edge), SWD header, labels
fp += rounded_rect(W, H, R, "F.Fab", 0.1)
fp += rect(-W / 2 - 1.0, -3.65, -W / 2 + 7.9, 3.65, "F.Fab", 0.1)
fp.append(text("USB-C", -W / 2 + 3.5, 0, "F.Fab", 0.8, 90))
fp += rect(10.2, -3.2, 12.2, 6.8, "F.Fab", 0.1)
fp.append(text("SWD", 11.2, -4.2, "F.Fab", 0.7))
fp.append(text("P1 (VCC side)", 0, Y_IN - 1.6, "F.Fab", 0.8))
fp.append(text("P2 (3V3 side)", 0, -Y_IN + 1.6, "F.Fab", 0.8))
fp.append(text("${REFERENCE}", 0, 0, "F.Fab", 1.0))
# Silk: outline slightly outside the board, broken where the USB-C sits, pin 1 marks for both headers
xs, ys = W / 2 + 0.12, H / 2 + 0.12
fp += [line((-xs + R, -ys), (xs - R, -ys), "F.SilkS", 0.12), line((xs, -ys + R), (xs, ys - R), "F.SilkS", 0.12),
       line((xs - R, ys), (-xs + R, ys), "F.SilkS", 0.12),
       line((-xs, ys - R), (-xs, 4.2), "F.SilkS", 0.12), line((-xs, -4.2), (-xs, -ys + R), "F.SilkS", 0.12)]
k = R * (1 - 0.70710678)
fp += [arc((-xs, -ys + R), (-xs + k, -ys + k), (-xs + R, -ys), "F.SilkS", 0.12),
       arc((xs - R, -ys), (xs - k, -ys + k), (xs, -ys + R), "F.SilkS", 0.12),
       arc((xs, ys - R), (xs - k, ys - k), (xs - R, ys), "F.SilkS", 0.12),
       arc((-xs + R, ys), (-xs + k, ys - k), (-xs, ys - R), "F.SilkS", 0.12)]
fp.append(text("1", X0 - 1.9, Y_IN, "F.SilkS", 0.8))
fp.append(text("25", X0 - 2.2, -Y_IN, "F.SilkS", 0.8))
# Courtyard: board plus 0.25 mm, extended on the left for the USB-C receptacle
fp += rect(-W / 2 - 1.25, -H / 2 - 0.25, W / 2 + 0.25, H / 2 + 0.25, "F.CrtYd", 0.05)
# Pads
for i in range(12):
    x = X0 + i * PITCH
    fp.append(pad(2 * i + 1, P1[2 * i], x, Y_IN, "rect" if i == 0 else "circle"))
    fp.append(pad(2 * i + 2, P1[2 * i + 1], x, Y_OUT))
    fp.append(pad(24 + 2 * i + 1, P2[2 * i], x, -Y_IN, "rect" if i == 0 else "circle"))
    fp.append(pad(24 + 2 * i + 2, P2[2 * i + 1], x, -Y_OUT))
fp.append("  (model \"${KIPRJMOD}/../hardware/kicad/3d/WeAct-STM32G47xCxTxCoreBoard_V10_3D.step\" "
          "(offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))")
fp.append(")")
(OUT / f"{LIB}.pretty").mkdir(parents=True, exist_ok=True)
(OUT / f"{LIB}.pretty" / f"{NAME}.kicad_mod").write_text("\n".join(fp) + "\n")

# ------------------------------------------------------------------- symbol
def ptype(name):
    if name == "VCC": return "power_in"
    if name == "3V3": return "power_out"
    if name == "GND": return "power_in"
    if name == "NC": return "no_connect"
    if name in ("VREF+", "VB"): return "passive"
    return "bidirectional"

pins = {i + 1: n for i, n in enumerate(P1)}
pins.update({25 + i: n for i, n in enumerate(P2)})
def order(prefix):
    return sorted((num for num, n in pins.items() if n.startswith(prefix) and n != "NC"),
                  key=lambda num: int("".join(c for c in pins[num][2:] if c.isdigit()) or 0))
left = ([n for n, v in pins.items() if v == "VCC"] + [n for n, v in pins.items() if v == "3V3"]
        + [n for n, v in pins.items() if v == "VREF+"] + [n for n, v in pins.items() if v == "VB"]
        + [n for n, v in pins.items() if v == "NRST"] + [n for n, v in pins.items() if v == "GND"]
        + order("PA"))
right = order("PB") + order("PC") + [n for n, v in pins.items() if v == "NC"]
assert sorted(left + right) == list(range(1, 49)), "every pad must appear once"
rows = max(len(left), len(right))
HALF = (rows + 1) * 2.54 / 2
HALF = round(HALF / 1.27) * 1.27
XW = 15.24

def spin(num, x, y, rot):
    name = pins[num]
    return (f"      (pin {ptype(name)} line (at {fmt(x)} {fmt(y)} {rot}) (length 2.54)\n"
            f"        (name \"{name}\" (effects (font (size 1.27 1.27))))\n"
            f"        (number \"{num}\" (effects (font (size 1.27 1.27)))))")

sym = [f"(kicad_symbol_lib (version 20231120) (generator \"kicad_symbol_editor\") (generator_version \"8.0\")",
       f"  (symbol \"{NAME}\" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)",
       f"    (property \"Reference\" \"U\" (at {fmt(-XW)} {fmt(HALF + 1.27)} 0) (effects (font (size 1.27 1.27)) (justify left)))",
       f"    (property \"Value\" \"{NAME}\" (at {fmt(-XW)} {fmt(-HALF - 1.27)} 0) (effects (font (size 1.27 1.27)) (justify left)))",
       f"    (property \"Footprint\" \"{LIB}:{NAME}\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"Datasheet\" \"https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"Description\" \"WeAct STM32G474 core board V1.0, LQFP48 variant, 2 x 12-pin headers. VCC 3.3 to 20 V in, 3V3 is the on-board LDO output (250 mA). PA13/PA14 are on the SWD header, PA11/PA12 also serve the USB-C.\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"ki_keywords\" \"WeAct STM32G474 core board module\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (symbol \"{NAME}_0_1\"",
       f"      (rectangle (start {fmt(-XW)} {fmt(HALF)}) (end {fmt(XW)} {fmt(-HALF)}) (stroke (width 0.254) (type default)) (fill (type background))))",
       f"    (symbol \"{NAME}_1_1\""]
for i, num in enumerate(left):
    sym.append(spin(num, -XW - 2.54, HALF - 2.54 * (i + 1), 0))
for i, num in enumerate(right):
    sym.append(spin(num, XW + 2.54, HALF - 2.54 * (i + 1), 180))
sym += ["    )", "  )", ")"]
(OUT / f"{LIB}.kicad_sym").write_text("\n".join(sym) + "\n")
print("wrote", OUT / f"{LIB}.pretty" / f"{NAME}.kicad_mod", "and", OUT / f"{LIB}.kicad_sym")
