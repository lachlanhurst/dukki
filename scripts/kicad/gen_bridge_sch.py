#!/usr/bin/env python3
"""Generate the first schematic of the bridge carrier PCB (docs/bridge-pcb.md).

One-shot bootstrap: once the schematic has been edited in KiCad, the .kicad_sch file is the
source and this script must not be run over it again (it refuses if the file exists, unless
--force is given).

Every pin connects through a net label, a power symbol or a no-connect flag placed on the pin
end, so there are no wires to route and ERC checks every connection.

Needs the KiCad standard symbol libraries, so it runs on the KiCad VM:
    python3 scripts/kicad/gen_bridge_sch.py [--kicad-share /usr/share/kicad] [--force]
Writes hardware/kicad/Dukki.kicad_sym (project symbols derived from KiCad's) and the project
in hardware/bridge/.
"""
import argparse, json, math, re, uuid
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--kicad-share", default="/usr/share/kicad")
ap.add_argument("--force", action="store_true")
args = ap.parse_args()
SYMDIR = Path(args.kicad_share) / "symbols"
LIBDIR = Path("hardware/kicad")
PROJ = Path("hardware/bridge")
SCH = PROJ / "bridge.kicad_sch"
if SCH.exists() and not args.force:
    raise SystemExit(f"{SCH} exists; it is the source now. Use --force to overwrite it.")

_NS = uuid.UUID("6f1d3b1e-2a8c-4b6e-9a51-0d9c4f0e7a10")
_seq = [0]
def u():
    """Deterministic IDs: the same script yields the same UUIDs, so the board's links to the
    schematic symbols survive a regeneration."""
    _seq[0] += 1
    return str(uuid.uuid5(_NS, f"bridge-sch-{_seq[0]}"))

def fmt(v):
    v = round(v, 4)
    return str(int(v)) if v == int(v) else f"{v:.4f}".rstrip("0").rstrip(".")

# ------------------------------------------------------------ library access
def top_blocks(text):
    """Yield (name, block text) for each top-level (symbol "name" ...) in a .kicad_sym file."""
    i, n = 0, len(text)
    while True:
        i = text.find("(symbol \"", i)
        if i < 0:
            return
        depth, j, instr = 0, i, False
        while j < n:
            c = text[j]
            if instr:
                if c == "\\": j += 1
                elif c == "\"": instr = False
            elif c == "\"": instr = True
            elif c == "(": depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        block = text[i:j + 1]
        yield re.match(r"\(symbol \"([^\"]+)\"", block).group(1), block
        i = j + 1

_libs = {}
def lib_block(lib, name):
    if lib not in _libs:
        p = LIBDIR / f"{lib}.kicad_sym" if (LIBDIR / f"{lib}.kicad_sym").exists() else SYMDIR / f"{lib}.kicad_sym"
        _libs[lib] = dict(top_blocks(p.read_text()))
    return _libs[lib][name]

PIN_RE = re.compile(r"\(pin\s+(\w+)\s+\w+\s*\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)"
                    r".*?\(name\s+\"([^\"]*)\".*?\(number\s+\"([^\"]*)\"", re.S)
def pins_of(block):
    """{number: (x, y, angle, name, type)} in library coordinates (y up)."""
    out = {}
    for m in PIN_RE.finditer(block):
        typ, x, y, a, name, num = m.groups()
        out[num] = (float(x), float(y), float(a or 0), name, typ)
    return out

def rename(block, old, new):
    """Rename a symbol and its unit sub-symbols."""
    block = block.replace(f"(symbol \"{old}\"", f"(symbol \"{new}\"", 1)
    return re.sub(rf"\(symbol \"{re.escape(old)}_(\d+_\d+)\"", lambda m: f'(symbol "{new}_{m.group(1)}"', block)

def set_prop(block, prop, value):
    return re.sub(rf"(\(property \"{prop}\" )\"[^\"]*\"", lambda m: m.group(1) + json.dumps(value), block, count=1)

# ---------------------------------------------------- project symbol library
# LMR51606: same SOT-23-6 pinout as KiCad's LMR50410 (CB GND FB EN VIN SW).
lmr = rename(lib_block("Regulator_Switching", "LMR50410"), "LMR50410", "LMR51606")
lmr = set_prop(lmr, "Value", "LMR51606")
lmr = set_prop(lmr, "Datasheet", "https://www.ti.com/lit/ds/symlink/lmr51606.pdf")
lmr = set_prop(lmr, "Description", "4-V to 65-V, 0.6-A synchronous buck converter, SOT-23-6")
lmr = set_prop(lmr, "ki_keywords", "buck step-down switching regulator 65V")
# P-MOSFET with TO-252 numbering: 1 G, 2 D (tab), 3 S.
q = rename(lib_block("Device", "Q_PMOS"), "Q_PMOS", "Q_PMOS_GDS")
q = re.sub(r"\(number \"G\"", "(number \"1\"", q)
q = re.sub(r"\(number \"D\"", "(number \"2\"", q)
q = re.sub(r"\(number \"S\"", "(number \"3\"", q)
q = set_prop(q, "Value", "Q_PMOS_GDS")
q = set_prop(q, "Description", "P-MOSFET, pin 1 gate, 2 drain, 3 source (TO-252, SOT-23 order)")
dukki = ["(kicad_symbol_lib (version 20231120) (generator \"gen_bridge_sch\") (generator_version \"10.0\")",
         "  " + lmr, "  " + q, ")"]
(LIBDIR / "Dukki.kicad_sym").write_text("\n".join(dukki) + "\n")
_libs.pop("Dukki", None)

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

def instance(lib_id, ref, value, x, y, rot, footprint="", lcsc="", in_bom=True, dnp=False,
             ref_at=None, val_at=None, hide_ref=False, hide_val=False, extra=None):
    lib, name = lib_id.split(":")
    block = lib_block(lib, name)
    used[lib_id] = block.replace(f"(symbol \"{name}\"", f"(symbol \"{lib_id}\"", 1)   # sub-symbols keep bare names
    pins = pins_of(block)
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
    s = [f"  (symbol (lib_id \"{lib_id}\") (at {fmt(x)} {fmt(y)} {rot}) (unit 1)",
         f"    (exclude_from_sim no) (in_bom {'yes' if in_bom else 'no'}) (on_board yes) (dnp {'yes' if dnp else 'no'})",
         f"    (uuid \"{u()}\")"] + props
    s += [f"    (pin \"{n}\" (uuid \"{u()}\"))" for n in pins]
    s += [f"    (instances (project \"bridge\" (path \"/{ROOT}\" (reference \"{ref}\") (unit 1))))", "  )"]
    items.append("\n".join(s))
    return pins

def power_value_at(value, x, y, away):
    """Value text just beyond the symbol: clear of the arrow, centred text."""
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

def part(lib_id, ref, value, x, y, nets, rot=None, **kw):
    """Place a part and connect pins. nets: {pin number or name: net or None for no-connect}."""
    lib, name = lib_id.split(":")
    pins = pins_of(lib_block(lib, name))
    if rot is None:            # two-terminal Device parts drawn vertically are turned horizontal
        ys = {round(v[1], 2) for v in pins.values()}
        rot = 90 if lib == "Device" and len(pins) == 2 and len(ys) == 2 else 0
    x, y = snap(x), snap(y)
    if rot == 90 and "ref_at" not in kw:
        kw["ref_at"], kw["val_at"] = (x, y - 3.81), (x, y + 3.81)
    instance(lib_id, ref, value, x, y, rot, **kw)
    bynum = {k: k for k in pins}
    byname = {v[3]: k for k, v in pins.items()}
    done = set()
    for key, net in nets.items():
        nums = [k for k, v in pins.items() if v[3] == key] if key not in bynum else [key]
        if not nums:
            raise SystemExit(f"{ref}: no pin {key!r} (have {sorted(pins)} / {sorted(byname)})")
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
        raise SystemExit(f"{ref}: pins not assigned: {sorted(missing)}")

def flag(net, x, y):
    """PWR_FLAG on a net, with the net attached at its pin."""
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

# Footprints and LCSC numbers
R0603, C0603, C0805 = "Resistor_SMD:R_0603_1608Metric", "Capacitor_SMD:C_0603_1608Metric", "Capacitor_SMD:C_0805_2012Metric"
F1206 = "Fuse:Fuse_1206_3216Metric"
RES = {"470k": "C23178", "100k": "C25803", "10k": "C25804", "120k": "C31850", "22k": "C31850", "1k": "C21190", "0R": "C21189"}
RES["120k"] = "C25808"
RES["130k"] = "C22795"
RES["1M"] = "C22935"
RES["2.2M"] = "C22938"
RES["220k"] = "C22961"
def R(ref, val, x, y, a, b, **kw):
    part("Device:R", ref, val, x, y, {"1": a, "2": b}, footprint=R0603, lcsc=RES[val], **kw)
def C(ref, val, x, y, a, b, fp=C0603, lcsc="", **kw):
    part("Device:C", ref, val, x, y, {"1": a, "2": b}, footprint=fp, lcsc=lcsc, **kw)

# 1. Battery input, main fuse, soft start, bulk -------------------------------------------
box(15, 15, 200, 112, "Battery in, main fuse, soft start",
    "XT30U-M hand fitted. Q1 ramps +BATT_SW over about 30 ms (C2 Miller ramp); C1 holds Q1 off at plug-in. docs/bridge-pcb.md 3.3")
part("Connector_Generic:Conn_01x02", "J1", "XT30U-M", 40.64, 50.8, {"1": "GND", "2": "VBAT_IN"},
     footprint="Connector_AMASS:AMASS_XT30U-M_1x02_P5.0mm_Vertical", in_bom=False,
     extra={"Note": "Male XT30U-M solder cup, fitted by hand. Pin 1 (square pad) GND, pin 2 +BATT, as the footprint's silkscreen marks them"})
part("Device:Fuse", "F1", "10A", 78.74, 33.02, {"1": "VBAT_IN", "2": "VBAT_F"}, footprint=F1206, lcsc="C41367232",
     extra={"MPN": "BHFUSE BSMD1206C-2100T, 10 A time lag, 32 V DC"})
part("Device:D_Zener", "D2", "SMBJ26A", 78.74, 45.72, {"1": "VBAT_F", "2": "GND"}, footprint="Diode_SMD:D_SMB", lcsc="C19077580")
R("R2", "470k", 78.74, 58.42, "VBAT_F", "Q1_G")
C("C1", "2.2u 50V", 78.74, 71.12, "VBAT_F", "Q1_G", fp=C0805, lcsc="C377773")
part("Device:D_Zener", "D1", "BZT52C15", 78.74, 83.82, {"1": "VBAT_F", "2": "Q1_G"}, footprint="Diode_SMD:D_SOD-123", lcsc="C19077412")
part("Dukki:Q_PMOS_GDS", "Q1", "NCE40P70K", 127, 40.64, {"1": "Q1_G", "2": "VBAT_SW", "3": "VBAT_F"}, rot=0,
     footprint="Package_TO_SOT_SMD:TO-252-2", lcsc="C130101", ref_at=(137.16, 38.1), val_at=(142.24, 43.18))
R("R1", "470k", 119.38, 58.42, "Q1_G", "PWR_LO")     # to ground through the power switch (Q2) or JP2
C("C2", "47n 50V", 119.38, 71.12, "Q1_G", "Q1_MILLER", lcsc="C1622")
R("R3", "1k", 119.38, 83.82, "Q1_MILLER", "VBAT_SW")
C("C3", "10u 50V X7R", 167.64, 33.02, "VBAT_SW", "GND", fp="Capacitor_SMD:C_1210_3225Metric", lcsc="C596319")
C("C4", "10u 50V X7R", 167.64, 45.72, "VBAT_SW", "GND", fp="Capacitor_SMD:C_1210_3225Metric", lcsc="C596319")
C("C5", "2.2u 50V", 167.64, 58.42, "VBAT_SW", "GND", fp=C0805, lcsc="C377773")
C("C6", "2.2u 50V", 167.64, 71.12, "VBAT_SW", "GND", fp=C0805, lcsc="C377773")
R("R4", "10k", 167.64, 83.82, "VBAT_SW", "LED_A")
part("Device:LED", "D3", "green", 167.64, 96.52, {"1": "GND", "2": "LED_A"}, footprint="LED_SMD:LED_0603_1608Metric", lcsc="C12624")
text("D3 lit: +BATT_SW is up, soft start done", 152.4, 105.41)
flag("VBAT_IN", 30.48, 104.14)
flag("GND", 45.72, 104.14)
flag("VBAT_SW", 60.96, 104.14)

# 2. Branches: servo segments and head ---------------------------------------------------
box(210, 15, 405, 112, "Branches: servo segments A to D, head",
    "Fuses time lag (BHFUSE BSMD1206C). Data lines: 0R links, room for a series resistor (hardware.md 5.6). PH 2.0 vertical through hole, hand fitted")
SEGS = [("A", "PB3, left leg"), ("B", "PB9, right leg"), ("C", "PA2, neck and head"), ("D", "PC10, not used")]
for i, (s, what) in enumerate(SEGS):
    y = 30.48 + i * 15.24
    part("Device:Fuse", f"F{2 + i}", "4A", 228.6, y, {"1": "VBAT_SW", "2": f"VSEG_{s}"}, footprint=F1206, lcsc="C41367227",
         extra={"MPN": "BHFUSE BSMD1206C-1400T, 4 A time lag, 72 V DC"})
    R(f"R{5 + i}", "0R", 269.24, y, f"SEG_{s}", f"SEG_{s}_C")
    part("Connector_Generic:Conn_01x03", f"J{2 + i}", f"Seg {s}", 355.6, y + 2.54,
         {"1": f"SEG_{s}_C", "2": f"VSEG_{s}", "3": "GND"},
         footprint="Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical", in_bom=False,
         extra={"MPN": "JST B3B-PH-K-S, through hole, fitted by hand"},
         ref_at=(358.14, y - 3.81), val_at=(365.76, y + 2.54))
    text(f"{s}: {what}", 375.92, y + 1.27)
y = 30.48 + 4 * 15.24
part("Device:Fuse", "F6", "2A", 228.6, y, {"1": "VBAT_SW", "2": "VHEAD"}, footprint=F1206, lcsc="C41367223",
     extra={"MPN": "BHFUSE BSMD1206C-1200T, 2 A time lag, 72 V DC"})
part("Connector_Generic:Conn_01x04", "J6", "Head", 355.6, y + 2.54,
     {"1": "VHEAD", "2": "GND", "3": "LINK_TX", "4": "LINK_RX"},
     footprint="Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", in_bom=False,
     extra={"MPN": "JST B4B-PH-K-S, through hole, fitted by hand"},
     ref_at=(358.14, y - 3.81), val_at=(365.76, y + 3.81))
text("Head buck feed and link,\nhardware.md 11.3", 375.92, y + 1.27)

# 3. 5 V buck ------------------------------------------------------------------------------
box(15, 115, 200, 178, "5 V buck for the WeAct board",
    "LMR51606X, 400 kHz. Values from TI Table 8-1; 130k/22k gives 5.53 V, about 5.2 V after D8. EN tied to VIN")
for i, (ref, val, fp, lc) in enumerate([("C7", "2.2u 50V", C0805, "C377773"), ("C8", "2.2u 50V", C0805, "C377773"),
                                          ("C10", "100n 50V", C0603, "C14663")]):
    C(ref, val, 45.72, 133.35 + i * 12.7, "VBAT_SW", "GND", fp=fp, lcsc=lc)
part("Dukki:LMR51606", "U1", "LMR51606XDBVR", 101.6, 147.32,
     {"CB": "BUCK_CB", "GND": "GND", "FB": "BUCK_FB", "EN": "VBAT_SW", "VIN": "VBAT_SW", "SW": "BUCK_SW"}, rot=0,
     footprint="Package_TO_SOT_SMD:SOT-23-6", lcsc="C21260865", ref_at=(101.6, 134.62), val_at=(101.6, 137.16))
C("C9", "100n 50V", 147.32, 133.35, "BUCK_CB", "BUCK_SW", lcsc="C14663")
part("Device:L", "L1", "33u", 147.32, 146.05, {"1": "BUCK_SW", "2": "5V_BUCK"}, footprint="Inductor_SMD:L_Sunlord_SWPA6045S",
     lcsc="C83451", extra={"MPN": "Sunlord SWPA6045S330MT"})
R("R9", "130k", 147.32, 158.75, "5V_BUCK", "BUCK_FB")
R("R10", "22k", 147.32, 171.45, "BUCK_FB", "GND")
C("C11", "22u 25V", 182.88, 139.7, "5V_BUCK", "GND", fp=C0805, lcsc="C45783")
C("C12", "22u 25V", 182.88, 152.4, "5V_BUCK", "GND", fp=C0805, lcsc="C45783")
flag("+5V", 182.88, 171.45)

# 4. Pack voltage --------------------------------------------------------------------------
box(210, 115, 300, 178, "Pack voltage to PA0", "Full scale 36 V (hardware.md 5.8)")
R("R11", "100k", 254, 137.16, "VBAT_SW", "VBAT_SENSE")
R("R12", "10k", 254, 149.86, "VBAT_SENSE", "GND")
C("C13", "100n 50V", 254, 162.56, "VBAT_SENSE", "GND", lcsc="C14663")

# 5. Mounting holes ------------------------------------------------------------------------
box(310, 115, 405, 140, "Mounting holes", "M2, one per corner, not connected")
for i in range(4):
    part("Mechanical:MountingHole", f"H{1 + i}", "M2", 325.12 + i * 20.32, 129.54, {}, rot=0,
         footprint="MountingHole:MountingHole_2.2mm_M2", in_bom=False)

# 5a. Backup 5 V ---------------------------------------------------------------------------
box(310, 146, 405, 178, "Buck output, backup 5 V",
    "D8 stops USB 5 V feeding back through the buck. Buck fails: cut JP1, external step-down to J9")
part("Device:D_Schottky", "D8", "B5819W", 335.28, 158.75, {"1": "5V_D", "2": "5V_BUCK"}, rot=0,
     footprint="Diode_SMD:D_SOD-123", lcsc="C8598", ref_at=(335.28, 155.575), val_at=(335.28, 161.925))
part("Jumper:SolderJumper_2_Bridged", "JP1", "cut to isolate buck", 335.28, 168.91, {"1": "5V_D", "2": "+5V"},
     rot=0, footprint="Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm", in_bom=False,
     ref_at=(335.28, 165.1), val_at=(335.28, 173.99))
part("Connector_Generic:Conn_01x04", "J9", "Ext 5V", 378.46, 160.02,
     {"1": "VBAT_SW", "2": "GND", "3": "+5V", "4": "GND"}, rot=0,
     footprint="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", in_bom=False,
     ref_at=(381.0, 154.94), val_at=(388.62, 160.02),
     extra={"Note": "Wire pads: 1 BAT+ (switched rail) and 2 GND to the external step-down input, 3 5V and 4 GND from its output"})
text("J9: 1 BAT+ out, 2 GND,\n3 5V in, 4 GND", 386.08, 165.1)

# 6. Bridge MCU ----------------------------------------------------------------------------
box(15, 184, 200, 284, "Bridge MCU: WeAct STM32G474 core board",
    "Soldered flush. Pin allocation: hardware.md 5.8. Unused pins not connected")
MCU = {"VCC": "+5V", "GND": "GND", "3V3": "+3V3",
       "PA9": "LINK_TX", "PA10": "LINK_RX", "PB3": "SEG_A", "PB9": "SEG_B", "PA2": "SEG_C", "PC10": "SEG_D",
       "PB13": "IMU_SCK", "PB14": "IMU_MISO", "PB15": "IMU_MOSI", "PA4": "IMU_CS", "PA8": "IMU_INT1",
       "PA0": "VBAT_SENSE", "PA15": "OLED_SCL", "PB7": "OLED_SDA", "PB5": "BTN",
       "PC4": "PWR_HOLD", "PA5": "PA5_IO"}
wpins = pins_of(lib_block("WeAct_STM32G474CoreBoard", "WeAct_STM32G474CoreBoard_QFN48"))
mnets = {num: MCU.get(v[3]) for num, v in wpins.items()}
part("WeAct_STM32G474CoreBoard:WeAct_STM32G474CoreBoard_QFN48", "U2", "WeAct STM32G474 QFN48", 106.68, 238.76, mnets,
     rot=0, footprint="WeAct_STM32G474CoreBoard:WeAct_STM32G474CoreBoard_QFN48", in_bom=False,
     ref_at=(124.46, 193.04), val_at=(124.46, 195.58))

# 7. Trunk IMU -----------------------------------------------------------------------------
box(210, 184, 295, 226, "Trunk IMU: LSM6DSV16X breakout", "SPI2, 4-wire. Soldered flush, 7-pin row only")
part("LSM6DSV16X_Breakout:LSM6DSV16X_Breakout", "U3", "LSM6DSV16X breakout", 251.46, 208.28,
     {"VCC": "+3V3", "GND": "GND", "SCL/SCLK": "IMU_SCK", "SDA/MOSI": "IMU_MOSI", "AD0/MISO": "IMU_MISO",
      "CS": "IMU_CS", "INT1": "IMU_INT1", "SDX": None, "SCX": None, "INT2": None}, rot=0,
     footprint="LSM6DSV16X_Breakout:LSM6DSV16X_Breakout", in_bom=False, ref_at=(241.3, 198.12), val_at=(241.3, 218.44))

# 8. Display and button --------------------------------------------------------------------
box(210, 232, 295, 284, "Display and button connectors", "PH 2.0 vertical through hole, hand fitted")
part("Connector_Generic:Conn_01x05", "J7", "OLED", 264.16, 248.92,
     {"1": "GND", "2": "+3V3", "3": "OLED_SCL", "4": "OLED_SDA", "5": None},
     footprint="Connector_JST:JST_PH_B5B-PH-K_1x05_P2.00mm_Vertical", in_bom=False,
     extra={"MPN": "JST B5B-PH-K-S, through hole, fitted by hand; 5 pins so the 4-pin head cable cannot fit"},
     ref_at=(266.7, 243.84), val_at=(276.86, 248.92))
part("Connector_Generic:Conn_01x02", "J8", "Power button", 264.16, 269.24, {"1": "BTN_A", "2": "BTN_B"},
     footprint="Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", in_bom=False,
     extra={"MPN": "JST B2B-PH-K-S, through hole, fitted by hand"},
     ref_at=(266.7, 264.16), val_at=(281.94, 269.24))

# 9. Soft power switch ----------------------------------------------------------------------
box(15, 292, 405, 400, "Soft power switch (docs/bridge-pcb.md 6.5)",
    "Button or PC4 turns Q2 on, which grounds R1 and lets the soft start turn Q1 on. Q2 off: Q5 turns Q4 on, "
    "snapping Q1 off. JP2 bridged = always on, as before. Off leaves about 20 uA of leakage")
# button input: pack-fed through R14; the contacts switch about 30 uA; D9 clamps the button side to 12 V,
# C14 and R15 debounce the sense; C15 on PWR_ON (below) stretches a tap until the bridge has booted.
# R14 only conducts while the button is held, so 220k costs nothing at rest and charges C14 + C15 quickly
R("R14", "220k", 50.8, 317.5, "VBAT_F", "BTN_A")
R("R15", "1M", 50.8, 330.2, "BTN_B", "GND")
C("C14", "1u 50V X7R", 50.8, 342.9, "BTN_B", "GND", fp=C0805, lcsc="C28323")
part("Device:D_Zener", "D9", "BZT52C12S", 50.8, 355.6, {"1": "BTN_B", "2": "GND"},
     footprint="Diode_SMD:D_SOD-323", lcsc="C19077447")
part("Device:D", "D12", "1N4148WS", 50.8, 368.3, {"1": "PWR_ON", "2": "BTN_B"},
     footprint="Diode_SMD:D_SOD-323", lcsc="C2128")
text("J8 (button) joins BTN_A and BTN_B", 30.48, 381.0)
# power hold from the bridge, and the switch itself
part("Device:D_Schottky", "D11", "BAT54WS", 116.84, 317.5, {"1": "PWR_ON", "2": "PWR_HOLD"},
     footprint="Diode_SMD:D_SOD-323", lcsc="C7502694")
R("R13", "1M", 116.84, 330.2, "PWR_ON", "GND")
# hold-up: with R13 about 1 s, so Q2 stays on through a bridge reset until the firmware re-asserts PC4
C("C15", "1u 50V X7R", 147.32, 330.2, "PWR_ON", "GND", fp=C0805, lcsc="C28323")
part("Transistor_FET:Q_NMOS_GSD", "Q2", "2N7002", 116.84, 350.52, {"1": "PWR_ON", "2": "GND", "3": "PWR_LO"},
     rot=0, footprint="Package_TO_SOT_SMD:SOT-23", lcsc="C8545", ref_at=(124.46, 347.98), val_at=(124.46, 353.06))
part("Jumper:SolderJumper_2_Open", "JP2", "bridge = always on", 116.84, 370.84, {"1": "PWR_LO", "2": "GND"}, rot=0,
     footprint="Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm", in_bom=False,
     ref_at=(116.84, 367.03), val_at=(116.84, 375.92))
# button sense for the bridge: PB5 sees only its pull-up and Q3, low while pressed (as on the breadboard)
part("Transistor_FET:Q_NMOS_GSD", "Q3", "2N7002", 187.96, 330.2, {"1": "BTN_B", "2": "GND", "3": "BTN"},
     rot=0, footprint="Package_TO_SOT_SMD:SOT-23", lcsc="C8545", ref_at=(195.58, 327.66), val_at=(195.58, 332.74))
text("Q3: PB5 reads low while the button is pressed", 167.64, 345.44)
# fast turn-off: with Q2 off, PWR_LO rises to Q1's gate, Q5 turns on and Q4 pulls Q1's gate to its source
R("R20", "1M", 256.54, 317.5, "PWR_LO", "Q5_G")
part("Device:D_Zener", "D13", "BZT52C12S", 256.54, 330.2, {"1": "Q5_G", "2": "GND"},
     footprint="Diode_SMD:D_SOD-323", lcsc="C19077447")
part("Transistor_FET:Q_NMOS_GSD", "Q5", "2N7002", 256.54, 350.52, {"1": "Q5_G", "2": "GND", "3": "Q4_PULL"},
     rot=0, footprint="Package_TO_SOT_SMD:SOT-23", lcsc="C8545", ref_at=(264.16, 347.98), val_at=(264.16, 353.06))
R("R21", "2.2M", 330.2, 317.5, "Q4_PULL", "Q4_B")
R("R22", "1M", 330.2, 330.2, "Q4_B", "VBAT_F")
part("Transistor_BJT:Q_PNP_BEC", "Q4", "MMBT3906", 330.2, 350.52, {"1": "Q4_B", "2": "VBAT_F", "3": "Q1_G"},
     rot=0, footprint="Package_TO_SOT_SMD:SOT-23", lcsc="C7420354", ref_at=(337.82, 347.98), val_at=(337.82, 353.06))

# 10. PA5 breakout ---------------------------------------------------------------------------
box(420, 292, 520, 340, "PA5 breakout", "Wire pads, bottom-left corner")
part("Connector_Generic:Conn_01x04", "J10", "PA5 breakout", 469.9, 314.96,
     {"1": "GND", "2": "+5V", "3": "+3V3", "4": "PA5_IO"}, rot=0,
     footprint="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", in_bom=False,
     ref_at=(472.44, 309.88), val_at=(482.6, 314.96))

# ------------------------------------------------------------------ write out
libsyms = "\n".join("    " + b.replace("\n", "\n    ") for b in used.values())
sch = [f"(kicad_sch (version 20251024) (generator \"gen_bridge_sch\") (generator_version \"10.0\")",
       f"  (uuid \"{ROOT}\")", "  (paper \"A2\")",
       "  (title_block (title \"Dukki bridge carrier\") (date \"10/10/2026\") (rev \"A, draft\")",
       "    (comment 1 \"Design note: docs/bridge-pcb.md\")",
       "    (comment 2 \"Bootstrapped by scripts/kicad/gen_bridge_sch.py; edit this file from now on\"))",
       "  (lib_symbols", libsyms, "  )"] + items + [
       "  (sheet_instances (path \"/\" (page \"1\")))", ")"]
PROJ.mkdir(parents=True, exist_ok=True)
SCH.write_text("\n".join(sch) + "\n")

libs = [("WeAct_STM32G474CoreBoard", "WeAct_STM32G474CoreBoard"), ("LSM6DSV16X_Breakout", "LSM6DSV16X_Breakout"),
        ("Dukki", "Dukki")]
(PROJ / "sym-lib-table").write_text("(sym_lib_table\n  (version 7)\n" + "".join(
    f"  (lib (name \"{n}\")(type \"KiCad\")(uri \"${{KIPRJMOD}}/../kicad/{f}.kicad_sym\")(options \"\")(descr \"\"))\n"
    for n, f in libs) + ")\n")
(PROJ / "fp-lib-table").write_text("(fp_lib_table\n  (version 7)\n" + "".join(
    f"  (lib (name \"{n}\")(type \"KiCad\")(uri \"${{KIPRJMOD}}/../kicad/{f}.pretty\")(options \"\")(descr \"\"))\n"
    for n, f in libs[:2]) + ")\n")
pro = PROJ / "bridge.kicad_pro"
if not pro.exists():
    pro.write_text(json.dumps({"meta": {"filename": "bridge.kicad_pro", "version": 1},
                               "sheets": [[ROOT, "Root"]]}, indent=2) + "\n")
print("wrote", SCH, "and", LIBDIR / "Dukki.kicad_sym")
