#!/usr/bin/env python3
"""Generate the KiCad footprint and symbol for the generic LSM6DSV16X breakout (the trunk IMU).

Dimensions are measured from the photo in docs/datasheets/lsm6dsv16xtr.png, scaled by the
7-pin row's 2.54 mm pitch (about 20 px/mm in both views):

- 7-pin row: 2.54 mm pitch, pad centres 1.27 mm in from the board's bottom edge, the row
  centred across the board. Exact enough to drill: both views agree.
- Board: about 17.8 x 15.0 mm (photo 17.3 to 17.9 by 14.6 to 15.0). Fab and courtyard only.
- 3-pin row (SDX, SCX, INT2) above pins 3 to 5, 12.0 mm from the 7-pin row: the photo gave
  about 12.0 and the user measured 12 mm on the part (11/10/2026). The pins are unused; the
  footprint gives them holes (pads 8 to 10, not connected) so header pins there support the
  breakout.
- Two mounting holes near the top corners, about 11.5 mm above the 7-pin row and 6.8 mm
  either side of centre. Fab layer only.

The breakout mounts with its components up and its labelled side down against the carrier,
so the top view is the photo's upper (component-side) view: VCC, the square pad, on the left.
The chip's X axis points right and Y up in that view (silkscreen arrows), Z out of the board.

Run from the repo root:  python3 scripts/kicad/gen_lsm6dsv16x_kicad.py
"""
import uuid
from pathlib import Path

NAME = "LSM6DSV16X_Breakout"
LIB = "LSM6DSV16X_Breakout"
OUT = Path("hardware/kicad")

W, H = 17.8, 15.0             # board outline, mm, measured from the photo
PITCH = 2.54
Y_ROW = H / 2 - 1.27          # 7-pin row, board centre as origin, y down
ROW3 = 12.0                   # 3-pin row distance above the 7-pin row, measured
HOLE_DX, HOLE_DY, HOLE_D = 6.8, 11.5, 2.2   # mounting holes, estimated, fab only
PAD, DRILL = 1.7, 1.0         # KiCad PinHeader_P2.54mm convention, as the WeAct footprint

# Top (component-side) view, left to right
PINS = ["VCC", "GND", "AD0/MISO", "SDA/MOSI", "SCL/SCLK", "CS", "INT1"]
ROW3_PINS = ["SDX", "SCX", "INT2"]   # above pins 3, 4, 5; pads 8 to 10, not connected, for support

def u():
    return f'(uuid "{uuid.uuid4()}")'

def fmt(v):
    return f"{v:.3f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)

# ---------------------------------------------------------------- footprint
def line(a, b, layer, width):
    return (f"  (fp_line (start {fmt(a[0])} {fmt(a[1])}) (end {fmt(b[0])} {fmt(b[1])}) "
            f"(stroke (width {width}) (type solid)) (layer \"{layer}\") {u()})")

def circle(c, r, layer, width):
    return (f"  (fp_circle (center {fmt(c[0])} {fmt(c[1])}) (end {fmt(c[0] + r)} {fmt(c[1])}) "
            f"(stroke (width {width}) (type solid)) (fill no) (layer \"{layer}\") {u()})")

def rect(x1, y1, x2, y2, layer, width):
    return [line((x1, y1), (x2, y1), layer, width), line((x2, y1), (x2, y2), layer, width),
            line((x2, y2), (x1, y2), layer, width), line((x1, y2), (x1, y1), layer, width)]

def text(s, x, y, layer, size=1.0, rot=0):
    return (f"  (fp_text user \"{s}\" (at {fmt(x)} {fmt(y)} {rot}) (layer \"{layer}\") {u()} "
            f"(effects (font (size {size} {size}) (thickness 0.12))))")

def prop(name, value, x, y, layer, hide=False):
    h = " (hide yes)" if hide else ""
    return (f"  (property \"{name}\" \"{value}\" (at {fmt(x)} {fmt(y)} 0) (layer \"{layer}\"){h} {u()} "
            f"(effects (font (size 1 1) (thickness 0.15))))")

def pad(num, name, x, y, shape="circle"):
    return (f"  (pad \"{num}\" thru_hole {shape} (at {fmt(x)} {fmt(y)}) (size {PAD} {PAD}) (drill {DRILL}) "
            f"(layers \"*.Cu\" \"*.Mask\") (remove_unused_layers no) (pinfunction \"{name}\") {u()})")

def px(i):
    return (i - 3) * PITCH       # pin 1 is i = 0, row centred

fp = [f"(footprint \"{NAME}\"", "  (version 20240108)", "  (generator \"pcbnew\")",
      "  (generator_version \"8.0\")", "  (layer \"F.Cu\")",
      "  (descr \"Generic LSM6DSV16X breakout, 7-pin 2.54 mm row (VCC GND AD0/MISO SDA/MOSI SCL/SCLK CS INT1), "
      "about 17.8 x 15.0 mm, mounted components up. Pads 8 to 10 are the 3-pin row (SDX SCX INT2), 12.0 mm above, "
      "not connected, for support. Outline and mounting holes are estimated from a photo.\")",
      "  (tags \"LSM6DSV16X IMU breakout module\")",
      prop("Reference", "REF**", 0, -(H / 2 + 1.2), "F.SilkS"),
      prop("Value", NAME, 0, H / 2 + 1.4, "F.Fab"),
      prop("Footprint", "", 0, 0, "F.Fab", hide=True),
      prop("Datasheet", "https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf", 0, 0, "F.Fab", hide=True),
      prop("Description", "", 0, 0, "F.Fab", hide=True),
      "  (attr through_hole)"]

# Fab: board outline, estimated 3-pin row and mounting holes, chip axes, labels
fp += rect(-W / 2, -H / 2, W / 2, H / 2, "F.Fab", 0.1)
fp.append(text("SDX SCX INT2 (support only)", 0, Y_ROW - ROW3 + 1.6, "F.Fab", 0.6))
for sx in (-1, 1):
    fp.append(circle((sx * HOLE_DX, Y_ROW - HOLE_DY), HOLE_D / 2, "F.Fab", 0.1))
ax, ay = 3.5, 1.0                         # axis arrows near the chip's place on the photo
fp += [line((ax, ay), (ax + 2.5, ay), "F.Fab", 0.12), line((ax + 2.5, ay), (ax + 2.0, ay - 0.3), "F.Fab", 0.12),
       line((ax + 2.5, ay), (ax + 2.0, ay + 0.3), "F.Fab", 0.12),
       line((ax, ay), (ax, ay - 2.5), "F.Fab", 0.12), line((ax, ay - 2.5), (ax - 0.3, ay - 2.0), "F.Fab", 0.12),
       line((ax, ay - 2.5), (ax + 0.3, ay - 2.0), "F.Fab", 0.12)]
fp.append(text("X", ax + 3.0, ay, "F.Fab", 0.7))
fp.append(text("Y", ax, ay - 3.0, "F.Fab", 0.7))
fp.append(text("${REFERENCE}", -3.0, -1.0, "F.Fab", 1.0))
for i, n in enumerate(PINS):
    fp.append(text(n.split("/")[0], px(i), Y_ROW - 2.0, "F.Fab", 0.5, 90))
# Silk: outline just outside the board, pin 1 mark
xs, ys = W / 2 + 0.12, H / 2 + 0.12
fp += rect(-xs, -ys, xs, ys, "F.SilkS", 0.12)
fp.append(text("1", px(0), Y_ROW - 1.9, "F.SilkS", 0.8))   # inside the outline, clear of neighbours either way up
# Courtyard: board plus 0.5 mm, since the outline is measured from a photo
fp += rect(-W / 2 - 0.5, -H / 2 - 0.5, W / 2 + 0.5, H / 2 + 0.5, "F.CrtYd", 0.05)
for i, n in enumerate(PINS):
    fp.append(pad(i + 1, n, px(i), Y_ROW, "rect" if i == 0 else "circle"))
for i, n in enumerate(ROW3_PINS):
    fp.append(pad(8 + i, n, px(2 + i), Y_ROW - ROW3))
fp.append("  (model \"${KIPRJMOD}/../kicad/3d/LSM6DSV16X_Breakout.step\" "
          "(offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))")
fp.append(")")
(OUT / f"{LIB}.pretty").mkdir(parents=True, exist_ok=True)
(OUT / f"{LIB}.pretty" / f"{NAME}.kicad_mod").write_text("\n".join(fp) + "\n")

# ------------------------------------------------------------------- symbol
TYPES = {"VCC": "power_in", "GND": "power_in", "INT1": "output", "AD0/MISO": "output",
         "SDX": "no_connect", "SCX": "no_connect", "INT2": "no_connect"}
ALL = PINS + ROW3_PINS
left = [1, 2, 8, 9, 10]        # VCC, GND, then the support-only 3-pin row
right = [5, 4, 3, 6, 7]        # SCL/SCLK, SDA/MOSI, AD0/MISO, CS, INT1
XW, HALF = 10.16, 7.62

def spin(num, x, y, rot):
    name = ALL[num - 1]
    return (f"      (pin {TYPES.get(name, 'input')} line (at {fmt(x)} {fmt(y)} {rot}) (length 2.54)\n"
            f"        (name \"{name}\" (effects (font (size 1.27 1.27))))\n"
            f"        (number \"{num}\" (effects (font (size 1.27 1.27)))))")

sym = [f"(kicad_symbol_lib (version 20231120) (generator \"kicad_symbol_editor\") (generator_version \"8.0\")",
       f"  (symbol \"{NAME}\" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)",
       f"    (property \"Reference\" \"U\" (at {fmt(-XW)} {fmt(HALF + 1.27)} 0) (effects (font (size 1.27 1.27)) (justify left)))",
       f"    (property \"Value\" \"{NAME}\" (at {fmt(-XW)} {fmt(-HALF - 1.27)} 0) (effects (font (size 1.27 1.27)) (justify left)))",
       f"    (property \"Footprint\" \"{LIB}:{NAME}\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"Datasheet\" \"https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"Description\" \"Generic LSM6DSV16X breakout. On-board regulator and I2C pull-ups; feed 3.3 V. SPI with CS low: SCL/SCLK is SCK, SDA/MOSI is SDI, AD0/MISO is SDO. SDX, SCX and INT2 (pins 8 to 10) are soldered for support only.\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
       f"    (property \"ki_keywords\" \"LSM6DSV16X IMU breakout module\" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))",
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
