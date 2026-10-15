#!/usr/bin/env python3
"""Generate the DukkiHead KiCad library for the head carrier PCB (docs/head-pcb.md).

Writes hardware/kicad/DukkiHead.kicad_sym and hardware/kicad/DukkiHead.pretty/ from:
- hardware/kicad/vendor/CM4IO_*: Raspberry Pi's CM4 IO board KiCad design (CM4IO-KiCAD.zip,
  datasheets.raspberrypi.com), the ComputeModule4-CM4 symbol and the Raspberry-Pi-4-Compute-Module
  footprint (both DF40C sockets and the four M2.5 holes), upgraded to the KiCad 10 format.
- hardware/kicad/vendor/EasyEDA_C19269369_*: the XUNPU FPC-1.0HF-15PWAR-H20 land pattern from
  LCSC's EasyEDA library (easyeda2kicad), upgraded the same way.
- KiCad's LMR50410 symbol, renumbered for the LMR51635 (needs the KiCad symbol libraries).
- The BMI088 module footprint and symbol, drawn here from hardware.md 6.2 and the module pictures
  in docs/datasheets/.

Edit this script, not the generated files. Runs on the KiCad VM:
    python3 scripts/kicad/gen_head_kicad.py [--kicad-share /usr/share/kicad]
"""
import argparse, re
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--kicad-share", default="/usr/share/kicad")
args = ap.parse_args()
LIBDIR = Path("hardware/kicad")
VENDOR = LIBDIR / "vendor"
PRETTY = LIBDIR / "DukkiHead.pretty"
PRETTY.mkdir(parents=True, exist_ok=True)


def sexpr_end(text, i):
    """Index of the parenthesis closing the expression that opens at text[i]."""
    depth, instr, j = 0, False, i
    while True:
        c = text[j]
        if instr:
            if c == "\\":
                j += 1
            elif c == "\"":
                instr = False
        elif c == "\"":
            instr = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return j
        j += 1


def drop_blocks(text, head):
    """Remove every top-level block starting with head, e.g. '(model '."""
    while (i := text.find(head)) >= 0:
        j = sexpr_end(text, i)
        text = text[:text.rfind("\n", 0, i) + 1] + text[j + 1:].lstrip(" \t").lstrip("\n")
    return text


def symbol_block(text, name):
    i = text.index(f"(symbol \"{name}\"")
    return text[i:sexpr_end(text, i) + 1]


def rename_symbol(block, old, new):
    block = block.replace(f"(symbol \"{old}\"", f"(symbol \"{new}\"", 1)
    return re.sub(rf"\(symbol \"{re.escape(old)}_(\d+_\d+)\"", lambda m: f'(symbol "{new}_{m.group(1)}"', block)


def set_prop(block, prop, value):
    return re.sub(rf"(\(property \"{prop}\"\s+)\"[^\"]*\"", lambda m: m.group(1) + f'"{value}"', block, count=1)


def set_descr(fp, text):
    if "(descr " in fp:
        return re.sub(r'\(descr "[^"]*"\)', f'(descr "{text}")', fp, count=1)
    return fp.replace('\t(layer "F.Cu")', f'\t(layer "F.Cu")\n\t(descr "{text}")', 1)


# ------------------------------------------------------------------ footprints
# Radxa CM4 (Pi CM4 form factor), from the CM4 IO board's footprint (origin at its bottom-left M2.5
# hole, pins 1 to 100 down the left, 101 to 200 down the right, holes 33 x 48 mm apart). Split into
# three so JLC places two sockets: one footprint per socket, origin at the socket's centre, and one
# for the module, origin at the module's centre, carrying its outline, courtyard and four holes. In
# the module footprint's frame the socket centres are at SOCKET_AT; the board script places them so.
src = (VENDOR / "CM4IO_Raspberry-Pi-4-Compute-Module.kicad_mod").read_text()
PAD_RE = re.compile(r'\(pad "(\d*)" (\w+) (\w+)\s*\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)\s*\(size ([-\d.]+) ([-\d.]+)\)')
pads = [m.groups() for m in PAD_RE.finditer(src)]
MOD_C = (16.5, -24.0)                      # module centre in the vendor frame (outline 40 x 55)
def centre(lo, hi):
    xs = [float(p[3]) for p in pads if p[0] and lo <= int(p[0]) <= hi]
    ys = [float(p[4]) for p in pads if p[0] and lo <= int(p[0]) <= hi]
    return ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
SOCKETS = {"1-100": (1, 100), "101-200": (101, 200)}
SOCKET_AT = {}
for name, (lo, hi) in SOCKETS.items():
    cx, cy = centre(lo, hi)
    SOCKET_AT[name] = (round(cx - MOD_C[0], 3), round(cy - MOD_C[1], 3))
    out = [f'(footprint "DF40C_Socket_CM4_{name}"\n\t(version 20241229)\n\t(generator "gen_head_kicad")\n'
           f'\t(layer "F.Cu")\n\t(descr "Hirose DF40C-100DS-0.4V(51) socket for Radxa CM4 / Pi CM4 pins {lo} to {hi}, '
           f'pads from Raspberry Pi\'s CM4 IO board footprint. Place at {SOCKET_AT[name]} from Radxa_CM4_Module\'s origin")\n'
           f'\t(attr smd)\n',
           f'\t(property "Reference" "REF**" (at 0 -12.5 0) (layer "F.SilkS")\n\t\t(effects (font (size 1 1) (thickness 0.15))))\n',
           f'\t(property "Value" "DF40C-100DS" (at 0 12.5 0) (layer "F.Fab")\n\t\t(effects (font (size 1 1) (thickness 0.15))))\n',
           '\t(fp_rect (start -1.89 -11.4) (end 1.89 11.4) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab"))\n']
    for num, kind, shape, x, y, a, w, h in pads:
        if num and lo <= int(num) <= hi:
            rot = f" {a}" if a else ""
            out.append(f'\t(pad "{num}" {kind} {shape} (at {float(x) - cx:.3f} {float(y) - cy:.3f}{rot}) (size {w} {h})\n'
                       f'\t\t(layers "F.Cu" "F.Paste" "F.Mask"))\n')
    out.append(")\n")
    (PRETTY / f"DF40C_Socket_CM4_{name}.kicad_mod").write_text("".join(out))
holes = [(float(p[3]) - MOD_C[0], float(p[4]) - MOD_C[1]) for p in pads if not p[0]]
out = ['(footprint "Radxa_CM4_Module"\n\t(version 20241229)\n\t(generator "gen_head_kicad")\n\t(layer "F.Cu")\n'
       '\t(descr "Radxa CM4 / Pi CM4 module, 40 x 55 mm, on 1.5 mm DF40C sockets: outline, courtyard (no parts under '
       'the module) and four 2.7 mm NPTH for M2.5. Sockets are separate footprints; origin at the module centre")\n'
       '\t(attr board_only exclude_from_pos_files exclude_from_bom)\n',
       '\t(property "Reference" "REF**" (at 0 -29 0) (layer "F.SilkS")\n\t\t(effects (font (size 1 1) (thickness 0.15))))\n',
       '\t(property "Value" "Radxa CM4" (at 0 0 0) (layer "F.Fab")\n\t\t(effects (font (size 1 1) (thickness 0.15))))\n',
       '\t(fp_rect (start -20 -27.5) (end 20 27.5) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab"))\n',

       '\t(fp_rect (start -20.25 -27.75) (end 20.25 27.75) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))\n',
       '\t(fp_text user "Wi-Fi U.FL" (at -15 24.5 0) (layer "F.Fab")\n\t\t(effects (font (size 0.8 0.8) (thickness 0.1))))\n']
for x, y in holes:
    out.append(f'\t(pad "" np_thru_hole circle (at {x:.3f} {y:.3f}) (size 2.7 2.7) (drill 2.7) (layers "*.Cu" "*.Mask"))\n')
out.append(")\n")
(PRETTY / "Radxa_CM4_Module.kicad_mod").write_text("".join(out))
(PRETTY / "Radxa_CM4_DF40C.kicad_mod").unlink(missing_ok=True)

# Camera FPC: XUNPU FPC-1.0HF-15PWAR-H20, 15 pins at 1.0 mm, dual contact, flip lock. The two
# anchor pads become MP so a Conn_01x15_MountingPin symbol ties them to ground.
fp = (VENDOR / "EasyEDA_C19269369_FPC-SMD_XUNPU_FPC-1.0HF-15PWAR-H20.kicad_mod").read_text()
fp = fp.replace('(footprint "FPC-SMD_XUNPU_FPC-1.0HF-15PWAR-H20"',
                '(footprint "FPC_XUNPU_FPC-1.0HF-15PWAR-H20_1x15-1MP_P1.0mm"', 1)
fp = fp.replace('(pad "16" smd', '(pad "MP" smd').replace('(pad "17" smd', '(pad "MP" smd')
fp = re.sub(r'(\(property "Value" )"[^"]*"', r'\1"FPC_XUNPU_FPC-1.0HF-15PWAR-H20"', fp, count=1)
fp = drop_blocks(fp, "(model ")
# drop the two pin-1 dots: turned to face the board edge, they land on or past it
fp = drop_blocks(fp, "(fp_circle")
fp = set_descr(fp, "XUNPU FPC-1.0HF-15PWAR-H20 (LCSC C19269369), 15-pin 1.0 mm FPC, dual contact, flip lock, "
                   "horizontal SMD. Land pattern from LCSC's EasyEDA library; contact pads on the -y side")
(PRETTY / "FPC_XUNPU_FPC-1.0HF-15PWAR-H20_1x15-1MP_P1.0mm.kicad_mod").write_text(fp)

# BMI088 module (hardware.md 6.2): 15 x 20 mm. 7-pin row VCC GND SCK SDA SDO csA csG (pads 1 to 7),
# 4-pin row INT4 INT3 INT2 INT1 (pads 8 to 11) 11 mm away (measured) and offset by 1.5 pitches, as
# the seller's pin diagram shows. Origin at pad 1, component side up, INT row towards -y.
P = 2.54
ROW = [("VCC", 0), ("GND", 1), ("SCK", 2), ("SDA", 3), ("SDO", 4), ("csA", 5), ("csG", 6)]
INT = [("INT4", 1.5), ("INT3", 2.5), ("INT2", 3.5), ("INT1", 4.5)]
X0, X1, Y0, Y1 = 3 * P - 10, 3 * P + 10, -13.0, 2.0      # board outline: 20 x 15 mm


def rect(layer, x0, y0, x1, y1, w):
    return (f'\t(fp_rect (start {x0:.3f} {y0:.3f}) (end {x1:.3f} {y1:.3f})\n'
            f'\t\t(stroke (width {w}) (type solid)) (fill no) (layer "{layer}"))\n')


def text(kind, value, x, y, layer, hide=False):
    h = " (hide yes)" if hide else ""
    return (f'\t(property "{kind}" "{value}" (at {x:.3f} {y:.3f} 0) (layer "{layer}"){h}\n'
            f'\t\t(effects (font (size 1 1) (thickness 0.15))))\n')


pads = []
for n, (name, i) in enumerate(ROW, 1):
    shape = "rect" if n == 1 else "circle"
    pads.append((str(n), name, i * P, 0.0, shape))
for n, (name, i) in enumerate(INT, len(ROW) + 1):
    pads.append((str(n), name, i * P, -11.0, "circle"))
out = ['(footprint "BMI088_Module"\n\t(version 20241229)\n\t(generator "gen_head_kicad")\n\t(layer "F.Cu")\n'
       '\t(descr "Generic BMI088 module (Shenzhen Module Studio), 15 x 20 mm, 7-pin and 4-pin rows at 2.54 mm, '
       '11 mm apart, soldered on short header pins. INT row offset 1.5 pitches from pad 1 (seller diagram)")\n'
       '\t(attr through_hole)\n',
       text("Reference", "REF**", 3 * P, Y0 - 1.5, "F.SilkS"),
       text("Value", "BMI088_Module", 3 * P, -5.5, "F.Fab"),
       rect("F.Fab", X0, Y0, X1, Y1, 0.1),
       rect("F.SilkS", X0 - 0.11, Y0 - 0.11, X1 + 0.11, Y1 + 0.11, 0.12),
       rect("F.CrtYd", X0 - 0.25, Y0 - 0.25, X1 + 0.25, Y1 + 0.25, 0.05),
       '\t(fp_line (start -1.4 1.4) (end -1.4 2.4) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))\n']
for num, name, x, y, shape in pads:
    out.append(f'\t(pad "{num}" thru_hole {shape} (at {x:.3f} {y:.3f}) (size 1.7 1.7) (drill 1)\n'
               f'\t\t(layers "*.Cu" "*.Mask") (pinfunction "{name}"))\n')
out.append(")\n")
(PRETTY / "BMI088_Module.kicad_mod").write_text("".join(out))

# A wide bridged solder jumper for the 5 V links (up to 3 A): 2 x 3 mm pads 0.6 mm apart joined by
# a 1.2 mm copper strip, mask open over the strip so a knife can cut it. A net tie, like KiCad's
# SolderJumper-2_P1.3mm_Bridged.
out = ['(footprint "SolderJumper-2_Wide_Bridged_3A"\n\t(version 20241229)\n\t(generator "gen_head_kicad")\n'
       '\t(layer "F.Cu")\n\t(descr "Bridged solder jumper for up to 3 A: 2 x 3 mm pads, 0.6 mm gap, 1.2 mm copper strip; '
       'cut the strip to open")\n\t(tags "net tie solder jumper bridged")\n',
       text("Reference", "REF**", 0, -2.6, "F.SilkS"), text("Value", "SolderJumper-2_Wide_Bridged_3A", 0, 2.7, "F.Fab"),
       '\t(attr exclude_from_pos_files exclude_from_bom allow_soldermask_bridges)\n\t(net_tie_pad_groups "1, 2")\n',
       '\t(fp_poly (pts (xy -0.4 -0.6) (xy 0.4 -0.6) (xy 0.4 0.6) (xy -0.4 0.6))\n'
       '\t\t(stroke (width 0) (type solid)) (fill yes) (layer "F.Cu"))\n',
       '\t(fp_rect (start -0.3 -1.5) (end 0.3 1.5) (stroke (width 0) (type default)) (fill yes) (layer "F.Mask"))\n',
       rect("F.SilkS", -2.6, -1.8, 2.6, 1.8, 0.12), rect("F.CrtYd", -2.75, -1.95, 2.75, 1.95, 0.05),
       '\t(pad "1" smd rect (at -1.3 0) (size 2 3) (layers "F.Cu" "F.Mask"))\n',
       '\t(pad "2" smd rect (at 1.3 0) (size 2 3) (layers "F.Cu" "F.Mask"))\n)\n']
(PRETTY / "SolderJumper-2_Wide_Bridged_3A.kicad_mod").write_text("".join(out))

# Wire pads for the SPI breakout: nine 1.5 x 2.5 mm SMD pads at 2.54 mm, no paste (wires are soldered
# by hand), names on the silkscreen under each pad. Placed flipped, on the bottom of the board.
SPI_NAMES = ("G", "5V", "3V3", "MO", "MI", "CK", "CS", "INT", "IO")
out = ['(footprint "SolderPads_1x09_P2.54mm_SPI"\n\t(version 20241229)\n\t(generator "gen_head_kicad")\n'
       '\t(layer "F.Cu")\n\t(descr "Nine wire pads at 2.54 mm for an SPI device: GND, 5V, 3V3, MOSI, MISO, CLK, CS, INT, '
       'spare GPIO. SMD, no paste; placed on the bottom")\n\t(attr smd exclude_from_pos_files exclude_from_bom)\n',
       text("Reference", "REF**", 4 * P, -2.6, "F.Fab"), text("Value", "SPI pads", 4 * P, 4.2, "F.Fab"),
       rect("F.CrtYd", -1.0, -1.5, 8 * P + 1.0, 3.3, 0.05)]
for n, name in enumerate(SPI_NAMES, 1):
    x = (n - 1) * P
    out.append(f'\t(pad "{n}" smd rect (at {x:.3f} 0) (size 1.5 2.5) (layers "F.Cu" "F.Mask"))\n')
    out.append(f'\t(fp_text user "{name}" (at {x:.3f} 2.4 0) (layer "F.SilkS")\n'
               f'\t\t(effects (font (size 0.8 0.8) (thickness 0.15))))\n')
out.append('\t(fp_line (start -1.05 -1.6) (end -1.05 1.6) (stroke (width 0.15) (type solid)) (layer "F.SilkS"))\n)\n')
(PRETTY / "SolderPads_1x09_P2.54mm_SPI.kicad_mod").write_text("".join(out))

# --------------------------------------------------------------------- symbols
# The CM4 IO symbol's two units become two symbols, one per socket footprint.
vend = symbol_block((VENDOR / "CM4IO_ComputeModule4-CM4.kicad_sym").read_text(), "ComputeModule4-CM4")
head = vend[:vend.index('(symbol "ComputeModule4-CM4_')]
cm4 = []
for unit, name in ((1, "1-100"), (2, "101-200")):
    i = vend.index(f'(symbol "ComputeModule4-CM4_{unit}_1"')
    sub = vend[i:sexpr_end(vend, i) + 1].replace(f'"ComputeModule4-CM4_{unit}_1"', f'"Radxa_CM4_{name}_1_1"', 1)
    blk = head.replace('(symbol "ComputeModule4-CM4"', f'(symbol "Radxa_CM4_{name}"', 1) + sub + "\n\t)"
    for k, v in {"Reference": "J", "Value": f"CM4 socket {name}", "Footprint": f"DukkiHead:DF40C_Socket_CM4_{name}",
                 "Description": f"Radxa CM4 (RK3576) pins {name} on a DF40C-100DS-0.4V(51) socket, Raspberry Pi "
                                "CM4 pin assignment"}.items():
        blk = set_prop(blk, k, v)
    cm4.append(blk)

# LMR51635: KiCad's LMR50410 drawing (SOT-23-6, pins CB GND FB EN VIN SW) renumbered to the
# LMR516x5 pinout, datasheet SLUSF64B table 5-1: 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 CB.
std = (Path(args.kicad_share) / "symbols" / "Regulator_Switching.kicad_sym").read_text()
lmr = rename_symbol(symbol_block(std, "LMR50410"), "LMR50410", "LMR51635")
NUM = {"FB": "1", "EN": "2", "VIN": "3", "GND": "4", "SW": "5", "CB": "6"}
lmr = re.sub(r'(\(name "(\w+)".*?\(number ")(\d+)"', lambda m: m.group(1) + NUM[m.group(2)] + '"', lmr, flags=re.S)
for k, v in {"Value": "LMR51635", "Footprint": "Package_TO_SOT_SMD:TSOT-23-6",
             "Datasheet": "https://www.ti.com/lit/ds/symlink/lmr51635.pdf",
             "Description": "4.3-V to 60-V, 3.5-A synchronous buck converter, SOT-23-6 (DDC)",
             "ki_keywords": "buck step-down switching regulator 60V"}.items():
    lmr = set_prop(lmr, k, v)

# MAX98357A: KiCad's symbol with the exposed pad (17) passive rather than unspecified, so tying it to
# ground raises no ERC warning.
amp = symbol_block((Path(args.kicad_share) / "symbols" / "Audio.kicad_sym").read_text(), "MAX98357A")
amp = re.sub(r'\(pin unspecified (line\s*\(at [^)]*\)\s*\(length [^)]*\)\s*\(name "PAD")', r'(pin passive \1', amp)
assert '(pin passive line' in amp

# BMI088: KiCad's symbol with SDO1 and SDO2 passive, since on this board they only strap the I2C
# addresses to a rail, which ERC rejects for output pins.
imu = symbol_block((Path(args.kicad_share) / "symbols" / "Sensor_Motion.kicad_sym").read_text(), "BMI088")
imu = re.sub(r'\(pin output (line\s*\(at [^)]*\)\s*\(length [^)]*\)\s*\(name "(SDO1|SDO2)")', r'(pin passive \1', imu)
assert imu.count("(pin passive line") >= 2


def pin(num, name, typ, x, y, ang):
    return (f'\t\t\t(pin {typ} line (at {x} {y} {ang}) (length 2.54)\n'
            f'\t\t\t\t(name "{name}" (effects (font (size 1.27 1.27))))\n'
            f'\t\t\t\t(number "{num}" (effects (font (size 1.27 1.27)))))\n')


def prop(k, v, x, y, hide=False):
    h = " (hide yes)" if hide else ""
    return f'\t\t(property "{k}" "{v}" (at {x} {y} 0) (effects (font (size 1.27 1.27)){h}))\n'


bmi = ['\t(symbol "BMI088_Module" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n',
       prop("Reference", "U", 0, 11.43), prop("Value", "BMI088_Module", 0, -11.43),
       prop("Footprint", "DukkiHead:BMI088_Module", 0, -13.97, True),
       prop("Datasheet", "https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf",
            0, -16.51, True),
       prop("Description", "Generic BMI088 6-axis IMU module, I2C/SPI, own 3.3 V regulator (hardware.md 6.2)",
            0, -19.05, True),
       '\t\t(symbol "BMI088_Module_0_1"\n\t\t\t(rectangle (start -7.62 10.16) (end 7.62 -10.16)\n'
       '\t\t\t\t(stroke (width 0.254) (type default)) (fill (type background))))\n',
       '\t\t(symbol "BMI088_Module_1_1"\n']
for n, (name, _) in enumerate(ROW, 1):
    typ = "power_in" if name in ("VCC", "GND") else "passive"
    bmi.append(pin(n, name, typ, -10.16, 7.62 - (n - 1) * 2.54, 0))
for n, (name, _) in enumerate(INT, len(ROW) + 1):
    bmi.append(pin(n, name, "passive", 10.16, 7.62 - (n - len(ROW) - 1) * 2.54, 180))
bmi.append("\t\t)\n\t)\n")

lib = ['(kicad_symbol_lib (version 20251024) (generator "gen_head_kicad") (generator_version "10.0")\n',
       "".join("\t" + b.replace("\n", "\n\t") + "\n" for b in cm4), "\t" + lmr.replace("\n", "\n\t") + "\n", "\t" + amp.replace("\n", "\n\t") + "\n", "\t" + imu.replace("\n", "\n\t") + "\n", "".join(bmi), ")\n"]
(LIBDIR / "DukkiHead.kicad_sym").write_text("".join(lib))
print("wrote", LIBDIR / "DukkiHead.kicad_sym", "and", PRETTY)
