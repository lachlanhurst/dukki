#!/usr/bin/env python3
"""Generate the first schematic of the head carrier PCB (docs/head-pcb.md).

One-shot bootstrap, as for the bridge: once the schematic has been edited in KiCad, the .kicad_sch
file is the source and this script must not be run over it again (it refuses if the file exists,
unless --force is given).

Every pin connects through a net label, a power symbol or a no-connect flag placed on the pin
end, so there are no wires to route and ERC checks every connection.

Needs the KiCad standard symbol libraries and hardware/kicad/DukkiHead.kicad_sym (made by
gen_head_kicad.py), so it runs on the KiCad VM:
    python3 scripts/kicad/gen_head_sch.py [--kicad-share /usr/share/kicad] [--force]
Writes the project in hardware/head/.
"""
import argparse, json, math, re, uuid
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--kicad-share", default="/usr/share/kicad")
ap.add_argument("--force", action="store_true")
args = ap.parse_args()
SYMDIR = Path(args.kicad_share) / "symbols"
LIBDIR = Path("hardware/kicad")
PROJ = Path("hardware/head")
SCH = PROJ / "head.kicad_sch"
if SCH.exists() and not args.force:
    raise SystemExit(f"{SCH} exists; it is the source now. Use --force to overwrite it.")

_NS = uuid.UUID("0b6c2f4e-81d3-4e0a-b7c5-3a9e5d1f2c64")
_seq = [0]
def u():
    """Deterministic IDs, so the board's links to the schematic survive a regeneration."""
    _seq[0] += 1
    return str(uuid.uuid5(_NS, f"head-sch-{_seq[0]}"))

def fmt(v):
    v = round(v, 4)
    return str(int(v)) if v == int(v) else f"{v:.4f}".rstrip("0").rstrip(".")

# ------------------------------------------------------------ library access
def sexpr_end(text, i):
    depth, instr, j = 0, False, i
    while True:
        c = text[j]
        if instr:
            if c == "\\": j += 1
            elif c == "\"": instr = False
        elif c == "\"": instr = True
        elif c == "(": depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return j
        j += 1

def top_blocks(text):
    """Yield (name, block text) for each top-level (symbol "name" ...) in a .kicad_sym file."""
    i = 0
    while (i := text.find("(symbol \"", i)) >= 0:
        j = sexpr_end(text, i)
        block = text[i:j + 1]
        yield re.match(r"\(symbol \"([^\"]+)\"", block).group(1), block
        i = j + 1

_libs = {}
def lib_block(lib, name):
    if lib not in _libs:
        p = LIBDIR / f"{lib}.kicad_sym" if (LIBDIR / f"{lib}.kicad_sym").exists() else SYMDIR / f"{lib}.kicad_sym"
        _libs[lib] = dict(top_blocks(p.read_text()))
    block = _libs[lib][name]
    m = re.search(r"\(extends \"([^\"]+)\"\)", block)
    if m:                      # a derived symbol: the parent's drawing and pins under this name
        parent = lib_block(lib, m.group(1))
        block = parent.replace(f"(symbol \"{m.group(1)}\"", f"(symbol \"{name}\"", 1)
        block = re.sub(rf"\(symbol \"{re.escape(m.group(1))}_(\d+_\d+)\"", lambda k: f'(symbol "{name}_{k.group(1)}"', block)
        child = _libs[lib][name]
        for pm in re.finditer(r"\(property \"([^\"]+)\"", child):      # the derived symbol's own fields win
            new = child[pm.start():sexpr_end(child, pm.start()) + 1]
            i = block.find(f"(property \"{pm.group(1)}\"")
            if i >= 0:
                block = block[:i] + new + block[sexpr_end(block, i) + 1:]
            else:
                k = block.find("(property ")
                block = block[:k] + new + "\n" + block[k:]
    return block

PIN_RE = re.compile(r"\(pin\s+(\w+)\s+\w+\s*\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)"
                    r".*?\(name\s+\"([^\"]*)\".*?\(number\s+\"([^\"]*)\"", re.S)
def pins_of(block, unit=1):
    """{number: (x, y, angle, name, type)} in library coordinates (y up), for one unit: the pins in
    the sub-symbols named _<unit>_<style> and _0_<style> (common to every unit)."""
    name = re.match(r"\(symbol \"([^\"]+)\"", block).group(1)
    out = {}
    for m in re.finditer(rf"\(symbol \"{re.escape(name)}_(\d+)_(\d+)\"", block):
        if int(m.group(1)) not in (0, unit):
            continue
        sub = block[m.start():sexpr_end(block, m.start()) + 1]
        for p in PIN_RE.finditer(sub):
            typ, x, y, a, pname, num = p.groups()
            out[num] = (float(x), float(y), float(a or 0), pname, typ)
    return out

# ------------------------------------------------------------- the schematic
ROOT = u()
items, used = [], {}
counter = {"pwr": 0}

def snap(v):
    return round(v / 1.27) * 1.27

def pin_end(x, y, rot, px, py):
    r = math.radians(rot)
    rx, ry = px * math.cos(r) - py * math.sin(r), px * math.sin(r) + py * math.cos(r)
    return round(x + rx, 4), round(y - ry, 4)

POWER = {"GND": "power:GND", "+5V": "power:+5V", "+3V3": "power:+3V3"}
POWER_AWAY = {"power:GND": 270, "power:+5V": 90, "power:+3V3": 90, "power:PWR_FLAG": 90}

def label(net, x, y, away):
    just = "left" if away in (0, 90) else "right"
    items.append(f"  (label \"{net}\" (at {fmt(x)} {fmt(y)} {away}) (fields_autoplaced yes)\n"
                 f"    (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid \"{u()}\"))")

def no_connect(x, y):
    items.append(f"  (no_connect (at {fmt(x)} {fmt(y)}) (uuid \"{u()}\"))")

def instance(lib_id, ref, value, x, y, rot, footprint="", lcsc="", in_bom=True, dnp=False, unit=1,
             ref_at=None, val_at=None, hide_ref=False, hide_val=False, extra=None):
    lib, name = lib_id.split(":")
    block = lib_block(lib, name)
    used[lib_id] = block.replace(f"(symbol \"{name}\"", f"(symbol \"{lib_id}\"", 1)   # sub-symbols keep bare names
    pins = pins_of(block, unit)
    ra = ref_at or (x, y - 5.08)
    va = val_at or (x, y + 5.08)
    def p(k, v, at, hide):
        h = " (hide yes)" if hide else ""
        return (f"    (property \"{k}\" {json.dumps(v)} (at {fmt(at[0])} {fmt(at[1])} {rot % 180})\n"
                f"      (effects (font (size 1.27 1.27)){h}))")
    props = [p("Reference", ref, ra, hide_ref), p("Value", value, va, hide_val),
             p("Footprint", footprint, (x, y), True), p("Datasheet", "", (x, y), True),
             p("Description", "", (x, y), True)]
    if lcsc:
        props.append(p("LCSC", lcsc, (x, y), True))
    for k, v in (extra or {}).items():
        props.append(p(k, v, (x, y), True))
    s = [f"  (symbol (lib_id \"{lib_id}\") (at {fmt(x)} {fmt(y)} {rot}) (unit {unit})",
         f"    (exclude_from_sim no) (in_bom {'yes' if in_bom else 'no'}) (on_board yes) (dnp {'yes' if dnp else 'no'})",
         f"    (uuid \"{u()}\")"] + props
    s += [f"    (pin \"{n}\" (uuid \"{u()}\"))" for n in pins]
    s += [f"    (instances (project \"head\" (path \"/{ROOT}\" (reference \"{ref}\") (unit {unit}))))", "  )"]
    items.append("\n".join(s))
    return pins

def power_value_at(value, x, y, away):
    if away in (0, 180):
        d = 3.3 + 0.55 * len(value)
        return (x + d if away == 0 else x - d, y)
    return (x, y - 4.6 if away == 90 else y + 4.6)

def power_sym(lib_id, value, x, y, away):
    counter["pwr"] += 1
    rot = (away - POWER_AWAY[lib_id]) % 360
    instance(lib_id, f"#PWR{counter['pwr']:03d}" if lib_id != "power:PWR_FLAG" else f"#FLG{counter['pwr']:03d}",
             value, x, y, rot, in_bom=False, hide_ref=True,
             val_at=power_value_at(value, x, y, away))

def part(lib_id, ref, value, x, y, nets, rot=None, unit=1, **kw):
    """Place a part and connect pins. nets: {pin number or name: net or None for no-connect}.
    A name matching several pins connects them all. Every pin of the unit must be assigned."""
    lib, name = lib_id.split(":")
    pins = pins_of(lib_block(lib, name), unit)
    if rot is None:            # two-terminal Device parts drawn vertically are turned horizontal
        ys = {round(v[1], 2) for v in pins.values()}
        rot = 90 if lib == "Device" and len(pins) == 2 and len(ys) == 2 else 0
    x, y = snap(x), snap(y)
    if rot == 90 and "ref_at" not in kw:
        kw["ref_at"], kw["val_at"] = (x, y - 3.81), (x, y + 3.81)
    instance(lib_id, ref, value, x, y, rot, unit=unit, **kw)
    done = set()
    for key, net in nets.items():
        nums = [key] if key in pins else [k for k, v in pins.items() if v[3] == key]
        if not nums:
            raise SystemExit(f"{ref}: no pin {key!r} (have {sorted(pins)})")
        for num in nums:
            px, py, a, _, _ = pins[num]
            ex, ey = pin_end(x, y, rot, px, py)
            away = int((a + rot + 180) % 360)
            if net is None:
                no_connect(ex, ey)
            elif net in POWER:
                power_sym(POWER[net], net, ex, ey, away)
            else:
                label(net, ex, ey, away)
            done.add(num)
    missing = set(pins) - done
    if missing:
        raise SystemExit(f"{ref}: pins not assigned: {sorted(missing, key=lambda n: (len(n), n))}")

def flag(net, x, y):
    x, y = snap(x), snap(y)
    power_sym("power:PWR_FLAG", "PWR_FLAG", x, y, 90)
    if net in POWER:
        power_sym(POWER[net], net, x, y, 270)
    else:
        label(net, x, y, 270)

def text(s, x, y, size=1.27, bold=False):
    b = " bold" if bold else ""
    items.append(f"  (text {json.dumps(s)} (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0)\n"
                 f"    (effects (font (size {size} {size}){b}) (justify left top)) (uuid \"{u()}\"))")

def box(x1, y1, x2, y2, title, note=""):
    items.append(f"  (rectangle (start {fmt(x1)} {fmt(y1)}) (end {fmt(x2)} {fmt(y2)})\n"
                 f"    (stroke (width 0.2) (type dash)) (fill (type none)) (uuid \"{u()}\"))")
    text(title, x1 + 2, y1 + 2, 2.0, True)
    if note:
        text(note, x1 + 2, y1 + 6)

# Footprints and LCSC numbers (all checked against JLC's library 12/10/2026)
R0603, C0603, C0805, C1206, C1210 = ("Resistor_SMD:R_0603_1608Metric", "Capacitor_SMD:C_0603_1608Metric",
                                     "Capacitor_SMD:C_0805_2012Metric", "Capacitor_SMD:C_1206_3216Metric",
                                     "Capacitor_SMD:C_1210_3225Metric")
RES = {"100R": "C22775", "330R": "C23138", "1k": "C21190", "4.7k": "C23162", "15k": "C22809",
       "82k": "C23254", "100k": "C25803", "1M": "C22935"}
CAP = {"100n 50V": (C0603, "C14663"), "2.2u 50V": (C0805, "C377773"), "10u 50V X7R": (C1210, "C596319"),
       "10u 25V": (C0805, "C15850"), "22u 25V": (C0805, "C45783"), "47u 10V": (C1206, "C96123")}
def R(ref, val, x, y, a, b, **kw):
    part("Device:R", ref, val, x, y, {"1": a, "2": b}, footprint=R0603, lcsc=RES[val], **kw)
def C(ref, val, x, y, a, b, **kw):
    fp, lc = CAP[val]
    part("Device:C", ref, val, x, y, {"1": a, "2": b}, footprint=fp, lcsc=lc, **kw)
def PH(ref, value, n, x, y, nets, note=""):
    part(f"Connector_Generic:Conn_01x{n:02d}", ref, value, x, y, nets, rot=0,
         footprint=f"Connector_JST:JST_PH_B{n}B-PH-K_1x{n:02d}_P2.00mm_Vertical", in_bom=False,
         extra={"MPN": f"JST B{n}B-PH-K-S, through hole, fitted by hand"},
         ref_at=(x + 2.54, y - 2.54 * (n // 2) - 3.81), val_at=(x + 10.16, y))
    if note:
        text(note, x + 15.24, y - 1.27)

# 1. Compute module -------------------------------------------------------------------------
box(10, 15, 185, 326, "Radxa CM4 on DF40C sockets",
    "Pin allocation: docs/head-pcb.md section 4. Unused pins not connected, as on the NANO-A")
CM4 = {"58": "I2C8_SDA", "56": "I2C8_SCL", "47": "UART7_TX", "45": "UART7_RX", "50": "FAN_PWM",
       "44": "EYE_DATA", "49": "SAI2_SCLK", "26": "SAI2_LRCK", "25": "SAI2_SDO", "27": "SAI2_SDI",
       "35": "CAM_SCL", "36": "CAM_SDA", "97": "CAM_GPIO", "78": "CM4_3V3", "84": "CM4_3V3",
       "128": "CSI_D0_N", "130": "CSI_D0_P", "134": "CSI_D1_N", "136": "CSI_D1_P",
       "140": "CSI_C_N", "142": "CSI_C_P"}
CM4.update({str(n): "+5V" for n in (77, 79, 81, 83, 85, 87)})
# SPI2 M1 broken out to wire pads (J13), for a future SPI device: CLK, MOSI, MISO, CS0, and two spare
# GPIOs for its interrupt and a reset or enable line (docs/head-pcb.md 6.2)
CM4.update({"31": "SPI2_CLK", "28": "SPI2_MOSI", "46": "SPI2_MISO", "54": "SPI2_CS0", "30": "SPI_INT", "34": "SPI_IO"})
for ref, name, (ux, uy) in (("J9", "1-100", (60.96, 96.52)), ("J10", "101-200", (-83.82, 241.3))):
    pins = pins_of(lib_block("DukkiHead", f"Radxa_CM4_{name}"))
    nets = {num: ("GND" if v[3] == "GND" else CM4.get(num)) for num, v in pins.items()}
    tx = ux if name == "1-100" else ux + 139.7
    part(f"DukkiHead:Radxa_CM4_{name}", ref, f"CM4 socket {name}", ux, uy, nets, rot=0,
         footprint=f"DukkiHead:DF40C_Socket_CM4_{name}", lcsc="C597931",
         extra={"MPN": "Hirose DF40C-100DS-0.4V(51)"}, ref_at=(tx, uy - (72.39 if name == "1-100" else 66.04)), val_at=(tx, uy + (74.93 if name == "1-100" else 66.04)))
part("Mechanical:MountingHole", "H5", "Radxa CM4", 162.56, 314.96, {}, rot=0,
     footprint="DukkiHead:Radxa_CM4_Module", in_bom=False,
     extra={"Note": "The module itself, fitted by hand: outline, keep-out and four M2.5 holes"})
text("GPIO_VREF (78) on the module's own 3.3 V output (84; 86 is the same rail, left open):\n3.3 V GPIO, as the NANO-A's H2 jumper", 15.24, 314.96)

# 2. Head bundle in, buck -------------------------------------------------------------------
box(195, 15, 395, 112, "Head bundle in, 5 V buck",
    "LMR51635Y, 1.1 MHz, 4.7 uH (TI table 8-2: 3.3 uH). 82k/15k sets 5.17 V. EN tied to VIN. F6 (2 A) on the bridge fuses this feed.\n"
    "JP1 joins the buck to the 5 V rail: cut it to feed 5 V from outside (J12). The feedback divider stays on the buck side")
PH("J1", "Head bundle", 4, 210.82, 45.72, {"1": "VBAT_IN", "2": "GND", "3": "UART7_RX", "4": "UART7_TX"},
   "1 +BATT, 2 GND, 3 bridge TX,\n4 bridge RX (bridge J6 order)")
part("Device:D_Zener", "D1", "SMBJ30A", 254, 71.12, {"1": "VBAT_IN", "2": "GND"},
     footprint="Diode_SMD:D_SMB", lcsc="C19077584")
C("C1", "10u 50V X7R", 254, 81.28, "VBAT_IN", "GND")
C("C2", "10u 50V X7R", 254, 91.44, "VBAT_IN", "GND")
C("C3", "2.2u 50V", 254, 101.6, "VBAT_IN", "GND")
C("C4", "100n 50V", 279.4, 101.6, "VBAT_IN", "GND")
part("DukkiHead:LMR51635", "U2", "LMR51635YDDCR", 304.8, 50.8,
     {"CB": "BUCK_CB", "GND": "GND", "FB": "BUCK_FB", "EN": "VBAT_IN", "VIN": "VBAT_IN", "SW": "BUCK_SW"}, rot=0,
     footprint="Package_TO_SOT_SMD:TSOT-23-6", lcsc="C45262770", ref_at=(304.8, 38.1), val_at=(304.8, 40.64))
C("C5", "100n 50V", 345.44, 35.56, "BUCK_CB", "BUCK_SW")
part("Device:L", "L1", "4.7u", 345.44, 48.26, {"1": "BUCK_SW", "2": "5V_BUCK"}, footprint="Inductor_SMD:L_Sunlord_SWPA6045S",
     lcsc="C78804", extra={"MPN": "Sunlord SWPA6045S4R7MT, 4.7 uH, 5.5 A saturation"})
R("R1", "82k", 345.44, 60.96, "5V_BUCK", "BUCK_FB")
R("R2", "15k", 345.44, 73.66, "BUCK_FB", "GND")
C("C6", "47u 10V", 375.92, 40.64, "5V_BUCK", "GND")
C("C7", "47u 10V", 375.92, 53.34, "5V_BUCK", "GND")
C("C8", "47u 10V", 375.92, 66.04, "5V_BUCK", "GND")
part("Jumper:SolderJumper_2_Bridged", "JP1", "cut for ext 5V", 345.44, 88.9, {"1": "5V_BUCK", "2": "+5V"}, rot=0,
     footprint="DukkiHead:SolderJumper-2_Wide_Bridged_3A", in_bom=False, ref_at=(345.44, 85.09), val_at=(345.44, 93.98))
flag("VBAT_IN", 215.9, 101.6)
flag("GND", 228.6, 101.6)
flag("+5V", 375.92, 93.98)

# 3. 3.3 V -----------------------------------------------------------------------------------
box(405, 15, 585, 70, "3.3 V", "AMS1117-3.3 from 5 V, as on the NANO-A. Camera, ToF, IMU, mic, I2C breakout")
part("Regulator_Linear:AMS1117-3.3", "U3", "AMS1117-3.3", 485.14, 40.64, {"VI": "+5V", "VO": "+3V3", "GND": "GND"},
     rot=0, footprint="Package_TO_SOT_SMD:SOT-223-3_TabPin2", lcsc="C6186", ref_at=(485.14, 31.75), val_at=(485.14, 34.29))
C("C9", "10u 25V", 449.58, 55.88, "+5V", "GND")
C("C10", "22u 25V", 520.7, 55.88, "+3V3", "GND")
C("C11", "100n 50V", 548.64, 55.88, "+3V3", "GND")

# 4. Speaker amplifier -----------------------------------------------------------------------
box(405, 76, 585, 145, "Speaker amplifier",
    "MAX98357A on SAI2. SD_MODE 1M to 5 V: (L+R)/2 mono. GAIN open = 9 dB; fit R4 (100k to GND) for 15 dB.\n"
    "JP3 feeds it: cut JP3 to use an external amplifier module on J11")
part("DukkiHead:MAX98357A", "U4", "MAX98357AETE+T", 472.44, 114.3,
     {"DIN": "SAI2_SDO", "BCLK": "SAI2_SCLK", "LRCLK": "SAI2_LRCK", "~{SD_MODE}": "AMP_SD", "GAIN_SLOT": "AMP_GAIN",
      "VDD": "AMP_5V", "GND": "GND", "PAD": "GND", "NC": None, "OUTP": "SPK_P", "OUTN": "SPK_N"}, rot=0,
     footprint="Package_DFN_QFN:TQFN-16-1EP_3x3mm_P0.5mm_EP1.23x1.23mm", lcsc="C910544",
     ref_at=(472.44, 97.79), val_at=(472.44, 100.33))
R("R3", "1M", 431.8, 132.08, "AMP_5V", "AMP_SD")
part("Jumper:SolderJumper_2_Bridged", "JP3", "cut for ext amp", 431.8, 90.17, {"1": "+5V", "2": "AMP_5V"}, rot=0,
     footprint="DukkiHead:SolderJumper-2_Wide_Bridged_3A", in_bom=False, ref_at=(431.8, 86.36), val_at=(431.8, 95.25))
R("R4", "100k", 515.62, 132.08, "AMP_GAIN", "GND", dnp=True)
C("C12", "10u 25V", 515.62, 93.98, "AMP_5V", "GND")
C("C13", "100n 50V", 541.02, 93.98, "AMP_5V", "GND")
flag("AMP_5V", 566.42, 93.98)
PH("J5", "Speaker", 2, 548.64, 116.84, {"1": "SPK_P", "2": "SPK_N"})
text("Bridge-tied output:\nneither lead to GND", 541.02, 124.46)

# 5. Fan and eye LED -------------------------------------------------------------------------
box(195, 118, 395, 178, "Fan and eye LED", "Fan: low side, PWM1_CH2 (pin 50). Eye: WS2812-type, SPI1 MOSI (pin 44), 3.3 V data")
R("R5", "100R", 223.52, 147.32, "FAN_PWM", "FAN_G")
R("R6", "100k", 243.84, 160.02, "FAN_G", "GND")
part("Transistor_FET:AO3400A", "Q1", "AO3400A", 266.7, 147.32, {"G": "FAN_G", "S": "GND", "D": "FAN_N"}, rot=0,
     footprint="Package_TO_SOT_SMD:SOT-23", lcsc="C20917", ref_at=(274.32, 144.78), val_at=(274.32, 149.86))
part("Device:D_Schottky", "D2", "B5819W", 297.18, 134.62, {"1": "+5V", "2": "FAN_N"},
     footprint="Diode_SMD:D_SOD-123", lcsc="C8598")
PH("J6", "Fan", 2, 320.04, 137.16, {"1": "+5V", "2": "FAN_N"}, "1 +5V, 2 FAN-")
R("R7", "330R", 297.18, 165.1, "EYE_DATA", "EYE_DIN")
PH("J4", "Eye LED", 3, 320.04, 165.1, {"1": "+5V", "2": "EYE_DIN", "3": "GND"}, "1 +5V, 2 DATA, 3 GND")

# 6. I2C8: IMU, ToF, breakout ------------------------------------------------------------------
box(195, 184, 395, 300, "I2C8: head IMU, ToF, breakout",
    "BMI088 on the board, I2C: PS and CSB1 high, CSB2 open (allowed in I2C mode); SDO1 and SDO2 high:\n"
    "accelerometer 0x19, gyroscope 0x69 (strapped so the pin rows need no vias between alternate pins).\n"
    "R8, R9: the bus pull-ups (the ToF breakout adds 10k). Cut JP4 and JP5 to use an external IMU module on J3")
part("DukkiHead:BMI088", "U5", "BMI088", 238.76, 236.22,
     {"VDD": "+3V3", "VDDIO": "+3V3", "GNDA": "GND", "GNDIO": "GND", "PS": "+3V3", "~{CSB1}": "+3V3",
      "~{CSB2}": None, "SDO1": "+3V3", "SDO2": "+3V3", "SCK/SCL": "IMU_SCL", "SDI/SDA": "IMU_SDA",
      "INT1": None, "INT2": None, "INT3": None, "INT4": None}, rot=0,
     footprint="Package_LGA:Bosch_LGA-16_4.5x3mm_P0.5mm_LayoutBorder7x1y_ClockwisePinNumbering", lcsc="C194919",
     ref_at=(238.76, 210.82), val_at=(238.76, 213.36))
C("C15", "100n 50V", 289.56, 233.68, "+3V3", "GND")
C("C16", "100n 50V", 289.56, 246.38, "+3V3", "GND")
text("C15 at VDD, C16 at VDDIO", 279.4, 251.46)
part("Jumper:SolderJumper_2_Bridged", "JP4", "cut for ext IMU", 223.52, 279.4, {"1": "I2C8_SDA", "2": "IMU_SDA"}, rot=0,
     footprint="Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm", in_bom=False,
     ref_at=(223.52, 275.59), val_at=(223.52, 284.48))
part("Jumper:SolderJumper_2_Bridged", "JP5", "cut for ext IMU", 254.0, 279.4, {"1": "I2C8_SCL", "2": "IMU_SCL"}, rot=0,
     footprint="Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm", in_bom=False,
     ref_at=(254.0, 275.59), val_at=(254.0, 284.48))
R("R8", "4.7k", 289.56, 208.28, "+3V3", "I2C8_SDA")
R("R9", "4.7k", 289.56, 220.98, "+3V3", "I2C8_SCL")
PH("J2", "ToF", 4, 330.2, 213.36, {"1": "GND", "2": "+3V3", "3": "I2C8_SDA", "4": "I2C8_SCL"}, "VL53L8CX breakout,\nQwiic order")
PH("J3", "I2C breakout", 4, 330.2, 241.3, {"1": "GND", "2": "+3V3", "3": "I2C8_SDA", "4": "I2C8_SCL"},
   "Spare I2C8, same order;\nexternal IMU module here")

# 7. Microphone ------------------------------------------------------------------------------
box(405, 151, 585, 190, "Microphone", "INMP441 on SAI2. L/R tied to GND on the mic side (left slot)")
PH("J7", "Microphone", 5, 449.58, 172.72, {"1": "+3V3", "2": "GND", "3": "SAI2_SCLK", "4": "SAI2_LRCK", "5": "SAI2_SDI"},
   "1 3V3, 2 GND, 3 SCK,\n4 WS, 5 SD")

# 8. Camera ----------------------------------------------------------------------------------
box(405, 196, 585, 290, "Camera FPC",
    "Pi 15-pin camera order. CAM0 lanes (Radxa CSI1), I2C6 with 4.7k pull-ups, CAM_GPIO enable. 100 ohm pairs")
part("Connector_Generic_MountingPin:Conn_01x15_MountingPin", "J8", "Camera", 477.52, 243.84,
     {"1": "GND", "2": "CSI_D0_N", "3": "CSI_D0_P", "4": "GND", "5": "CSI_D1_N", "6": "CSI_D1_P", "7": "GND",
      "8": "CSI_C_N", "9": "CSI_C_P", "10": "GND", "11": "CAM_GPIO", "12": None, "13": "CAM_SCL", "14": "CAM_SDA",
      "15": "+3V3", "MP": "GND"}, rot=0,
     footprint="DukkiHead:FPC_XUNPU_FPC-1.0HF-15PWAR-H20_1x15-1MP_P1.0mm", lcsc="C19269369",
     extra={"MPN": "XUNPU FPC-1.0HF-15PWAR-H20, dual contact"}, ref_at=(480.06, 220.98), val_at=(480.06, 266.7))
R("R10", "4.7k", 538.48, 233.68, "+3V3", "CAM_SCL")
R("R11", "4.7k", 538.48, 246.38, "+3V3", "CAM_SDA")
C("C14", "100n 50V", 538.48, 259.08, "+3V3", "GND")
text("Pin 12 (camera clock) not connected: the module has its own oscillator", 431.8, 279.4)

# 9. Power LED, mounting holes ----------------------------------------------------------------
box(195, 306, 395, 338, "Power LED, mounting holes", "M2 holes, one per corner, not connected")
R("R12", "1k", 213.36, 325.12, "+5V", "LED_A")
part("Device:LED", "D3", "green", 233.68, 325.12, {"1": "GND", "2": "LED_A"},
     footprint="LED_SMD:LED_0603_1608Metric", lcsc="C12624")
for i in range(4):
    part("Mechanical:MountingHole", f"H{1 + i}", "M2", 274.32 + i * 25.4, 325.12, {}, rot=0,
         footprint="MountingHole:MountingHole_2.2mm_M2", in_bom=False)

# 10. External fallbacks ----------------------------------------------------------------------
box(405, 296, 585, 380, "External fallbacks",
    "Wire pads, 2.54 mm. J12: cut JP1 and feed 5 V from an external step-down. J11: cut JP3 and wire an\n"
    "amplifier module, pins in the generic module's order. J3 (section 6) takes an external IMU module")
part("Connector_Generic:Conn_01x04", "J12", "Ext 5V", 449.58, 330.2,
     {"1": "VBAT_IN", "2": "GND", "3": "+5V", "4": "GND"}, rot=0,
     footprint="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", in_bom=False,
     ref_at=(452.12, 322.58), val_at=(459.74, 330.2),
     extra={"Note": "Wire pads: 1 B+ and 2 GND to the external step-down input, 3 5V and 4 GND from its output"})
text("J12: 1 B+ out, 2 GND,\n3 5V in, 4 GND", 467.36, 327.66)
part("Connector_Generic:Conn_01x07", "J11", "Ext amp", 449.58, 360.68,
     {"1": "SAI2_LRCK", "2": "SAI2_SCLK", "3": "SAI2_SDO", "4": None, "5": None, "6": "GND", "7": "+5V"}, rot=0,
     footprint="Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical", in_bom=False,
     ref_at=(452.12, 349.25), val_at=(459.74, 360.68),
     extra={"Note": "Wire pads in the generic MAX98357A module's order: LRC, BCLK, DIN, GAIN, SD, GND, Vin"})
text("J11: LRC, BCLK, DIN, GAIN (open),\nSD (open: module's own pull-up), GND, Vin", 467.36, 358.14)

# 11. SPI breakout ----------------------------------------------------------------------------
box(195, 344, 395, 400, "SPI breakout",
    "SPI2 M1 for a future SPI device, untested. Wire pads on the bottom, under the module, along the top edge.\n"
    "CLK GPIO12 (CM4 31), MOSI GPIO13 (28), MISO GPIO22 (46), CS0 GPIO4 (54), INT GPIO6 (30), spare GPIO5 (34)")
part("Connector_Generic:Conn_01x09", "J13", "SPI pads", 238.76, 375.92,
     {"1": "GND", "2": "+5V", "3": "+3V3", "4": "SPI2_MOSI", "5": "SPI2_MISO", "6": "SPI2_CLK", "7": "SPI2_CS0",
      "8": "SPI_INT", "9": "SPI_IO"}, rot=0,
     footprint="DukkiHead:SolderPads_1x09_P2.54mm_SPI", in_bom=False, ref_at=(241.3, 361.95), val_at=(248.92, 375.92),
     extra={"Note": "Bottom-side wire pads: G, 5V, 3V3, MO, MI, CK, CS, INT, IO"})
text("J13: 1 GND, 2 5V, 3 3V3, 4 MOSI, 5 MISO,\n6 CLK, 7 CS0, 8 INT, 9 spare GPIO", 256.54, 373.38)

# ------------------------------------------------------------------ write out
libsyms = "\n".join("    " + b.replace("\n", "\n    ") for b in used.values())
sch = [f"(kicad_sch (version 20251024) (generator \"gen_head_sch\") (generator_version \"10.0\")",
       f"  (uuid \"{ROOT}\")", "  (paper \"A2\")",
       "  (title_block (title \"Dukki head carrier\") (date \"12/10/2026\") (rev \"A, draft\")",
       "    (comment 1 \"Design note: docs/head-pcb.md\")",
       "    (comment 2 \"Bootstrapped by scripts/kicad/gen_head_sch.py; edit this file from now on\"))",
       "  (lib_symbols", libsyms, "  )"] + items + [
       "  (sheet_instances (path \"/\" (page \"1\")))", ")"]
PROJ.mkdir(parents=True, exist_ok=True)
SCH.write_text("\n".join(sch) + "\n")

(PROJ / "sym-lib-table").write_text(
    "(sym_lib_table\n  (version 7)\n"
    "  (lib (name \"DukkiHead\")(type \"KiCad\")(uri \"${KIPRJMOD}/../kicad/DukkiHead.kicad_sym\")(options \"\")(descr \"\"))\n)\n")
(PROJ / "fp-lib-table").write_text(
    "(fp_lib_table\n  (version 7)\n"
    "  (lib (name \"DukkiHead\")(type \"KiCad\")(uri \"${KIPRJMOD}/../kicad/DukkiHead.pretty\")(options \"\")(descr \"\"))\n)\n")
pro = PROJ / "head.kicad_pro"
if not pro.exists():
    pro.write_text(json.dumps({"meta": {"filename": "head.kicad_pro", "version": 1},
                               "sheets": [[ROOT, "Root"]]}, indent=2) + "\n")
print("wrote", SCH)
