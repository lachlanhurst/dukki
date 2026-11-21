#!/usr/bin/env python3
"""Generate docs/pictures/dukki-hardware.svg, the README's hardware block diagram.

Run from anywhere: python3 scripts/docs/hardware_diagram.py
Coordinates are laid out by hand; render the SVG and look at it after any change.
"""
from pathlib import Path

SVG = Path(__file__).resolve().parents[2] / "docs" / "pictures" / "dukki-hardware.svg"

W, H = 880, 1285

C_TEXT = "#1f2328"
C_SUB = "#57606a"
C_BOX = "#9aa4b2"
C_SIG = "#5b6570"
C_BATT = "#d1453b"
C_BATT_T = "#d1453b"
C_5V = "#e0901e"
C_5V_T = "#a86a10"
C_BUS = "#2f6fd6"
C_LINK = "#8a4fd0"
C_PANEL = "#d4d9e0"

out = []


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, size=11, anchor="start", bold=False, fill=C_SUB):
    w = "bold" if bold else "normal"
    out.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
               f'font-weight="{w}" fill="{fill}">{esc(s)}</text>')


def rect(x, y, w, h, fill="#ffffff", stroke=C_BOX, rx=6, dash=None, sw=1):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    s = f' stroke-width="{sw}"' if sw != 1 else ""
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
               f'fill="{fill}" stroke="{stroke}"{d}{s}/>')


def line(pts, color=C_SIG, width=2.2, arrow=None):
    p = " ".join(f"{x},{y}" for x, y in pts)
    m = f' marker-end="url(#arrow-{arrow})"' if arrow else ""
    out.append(f'<polyline points="{p}" fill="none" stroke="{color}" '
               f'stroke-width="{width}" stroke-linejoin="round"{m}/>')


def dot(x, y, color=C_SIG):
    out.append(f'<circle cx="{x}" cy="{y}" r="3.5" fill="{color}"/>')


def box(x, y, w, h, title, subs=(), stroke=C_BOX, tsize=13):
    """A part: white box, bold title, grey subtitle lines, centred."""
    rect(x, y, w, h, stroke=stroke)
    n = len(subs)
    block = tsize + 15 * n
    ty = y + (h - block) / 2 + tsize - 1.5
    cx = x + w / 2
    text(cx, ty, title, size=tsize, anchor="middle", bold=True, fill=C_TEXT)
    for i, s in enumerate(subs):
        text(cx, ty + 15 * (i + 1), s, anchor="middle")


def board(x, y, w, h, name, note):
    """A PCB outline: pale green, dashed edge, name in its top-left corner."""
    rect(x, y, w, h, fill="#f3f8ef", stroke="#6f9a5a", rx=4, dash="6 3", sw=1.4)
    text(x + 12, y + 18, name, size=12, bold=True, fill="#4c7a38")
    text(x + 12, y + 32, note, size=10, fill="#4c7a38")


out.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" '
           f'height="{H}" font-family="-apple-system, BlinkMacSystemFont, \'Segoe UI\', '
           f'Helvetica, Arial, sans-serif">')
out.append("<defs>")
for c in ("d1453b", "e0901e"):
    out.append(f'<marker id="arrow-{c}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
               f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
               f'fill="#{c}"/></marker>')
out.append("</defs>")
out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')

# ---------------------------------------------------------------- panels
rect(20, 20, 840, 435, fill="#fbf6ec", stroke=C_PANEL, rx=10)
text(36, 42, "HEAD", size=13, bold=True)
rect(20, 470, 840, 365, fill="#edf3fb", stroke=C_PANEL, rx=10)
text(36, 492, "TRUNK", size=13, bold=True)
rect(20, 850, 840, 305, fill="#edf7f0", stroke=C_PANEL, rx=10)
text(36, 872, "SERVOS", size=13, bold=True)

# ---------------------------------------------------------------- head
BX, BY, BW, BH = 190, 52, 490, 388
board(BX, BY, BW, BH, "head_ext carrier PCB", "80 x 55 mm, the CM4 plugs straight in")

# Radxa CM4
CMX, CMY, CMW, CMH = 270, 100, 200, 290
rect(CMX, CMY, CMW, CMH, fill="#fffdf8", stroke="#c08a2a")
cx = CMX + CMW / 2
text(cx, 205, "Radxa CM4", size=15, anchor="middle", bold=True, fill=C_TEXT)
for i, s in enumerate(["RK3576, 2 GB LPDDR4x,", "32 GB eMMC",
                       "Armbian, Rockchip 6.1",
                       "vendor kernel", "",
                       "Wi-Fi 6 / BT 5.4 antenna",
                       "USB flashing on test pads"]):
    text(cx, 222 + 15 * i, s, anchor="middle")

# off-board, left of the carrier
LX, LW = 36, 130
box(LX, 110, LW, 50, "IMX219 camera", ["15-pin FPC"])
line([(LX + LW, 135), (CMX, 135)])
text(230, 129, "CSI, 2 lanes", size=10, anchor="middle", fill=C_SIG)

box(LX, 190, LW, 50, "VL53L8CX ToF", ["0x29"])
line([(LX + LW, 215), (CMX, 215)])
text(230, 209, "I2C8", size=10, anchor="middle", fill=C_SIG)

box(LX, 270, LW, 50, "INMP441 mic", [])
line([(LX + LW, 295), (CMX, 295)])
text(230, 289, "I2S in", size=10, anchor="middle", fill=C_SIG)

# on the carrier, right of the CM4
OX, OW = 540, 125
RX, RW = 700, 135          # off-board, right of the carrier
GAP_L = (CMX + CMW + OX) / 2

box(OX, 100, OW, 50, "BMI088 IMU", ["module, 0x18 / 0x68"], tsize=12)
line([(CMX + CMW, 125), (OX, 125)])
text(GAP_L, 119, "I2C8", size=10, anchor="middle", fill=C_SIG)

box(OX, 162, OW, 50, "MAX98357A", ["speaker amp"], tsize=12)
line([(CMX + CMW, 187), (OX, 187)])
text(GAP_L, 181, "I2S out", size=10, anchor="middle", fill=C_SIG)
box(RX, 162, RW, 50, "3 W speaker", [])
line([(OX + OW, 187), (RX, 187)])

box(OX, 224, OW, 50, "Fan MOSFET", [], tsize=12)
line([(CMX + CMW, 249), (OX, 249)])
text(GAP_L, 243, "PWM", size=10, anchor="middle", fill=C_SIG)
box(RX, 224, RW, 50, "Head fan", ["5 V, on / off"])
line([(OX + OW, 249), (RX, 249)])

box(RX, 286, RW, 50, "Eye LED", ["WS2812"])
line([(CMX + CMW, 311), (RX, 311)])
text((OX + OX + OW) / 2, 305, "SPI as WS2812 data", size=10, anchor="middle", fill=C_SIG)

box(OX, 340, OW, 64, "Head buck", ["+BATT to 5 V, 3 A", "also amp, LED, fan"], stroke=C_5V, tsize=12)
line([(OX, 372), (CMX + CMW + 2, 372)], color=C_5V, arrow="e0901e")
text(GAP_L, 366, "5 V", anchor="middle", fill=C_5V_T)

# ---------------------------------------------------------------- trunk
PX, PY, PW, PH = 190, 502, 510, 300
board(PX, PY, PW, PH, "Bridge carrier PCB", "45 x 80 mm, rev B")

# power path, top row
box(32, 548, 116, 50, "6S LiPo", ["21.0 to 25.2 V"], stroke=C_BATT)
line([(148, 573), (203, 573)], color=C_BATT, arrow="d1453b")
text(168, 566, "XT30", size=10, anchor="middle", fill=C_BATT_T)

box(205, 548, 100, 50, "Main fuse", ["10 A, TVS"], stroke=C_BATT, tsize=12)
line([(305, 573), (318, 573)], color=C_BATT, arrow="d1453b")
box(320, 548, 110, 50, "Power switch", ["soft start P-FET"], stroke=C_BATT, tsize=12)
line([(430, 573), (538, 573)], color=C_BATT, arrow="d1453b")
text(500, 566, "+BATT_SW", size=10, anchor="middle", fill=C_BATT_T)
box(540, 541, 145, 64, "Branch fuses", ["4 A per segment", "2 A head feed"], stroke=C_BATT, tsize=12)

# left column
box(205, 625, 110, 44, "5 V buck", ["0.6 A"], stroke=C_5V, tsize=12)
line([(335, 598), (335, 612), (260, 612), (260, 623)], color=C_BATT, arrow="d1453b")
box(205, 682, 110, 44, "Pack voltage", ["100k / 10k"], stroke=C_BATT, tsize=12)
box(205, 739, 110, 50, "LSM6DSV16X", ["trunk IMU, SFLP"], tsize=12)

# WeAct
WX, WY, WW, WH = 355, 625, 190, 164
rect(WX, WY, WW, WH, fill="#f8fbff", stroke=C_BUS)
wcx = WX + WW / 2
text(wcx, 690, "STM32G474", size=14, anchor="middle", bold=True, fill=C_TEXT)
text(wcx, 706, "WeAct core board", anchor="middle")
text(wcx, 720, "QFN48, 168 MHz", anchor="middle")
text(wcx, 734, "USB-C: console, DFU", anchor="middle")

line([(315, 647), (WX - 1, 647)], color=C_5V, arrow="e0901e")
text(325, 641, "5 V", size=10, anchor="middle", fill=C_5V_T)
line([(315, 704), (WX, 704)])
text(335, 698, "ADC", size=10, anchor="middle", fill=C_SIG)
line([(315, 764), (WX, 764)])
text(335, 758, "SPI", size=10, anchor="middle", fill=C_SIG)

# hold line from the bridge to the power switch
line([(395, WY), (395, 598)])
text(400, 616, "PC4 hold", size=10, fill=C_SIG)

# off-board, right of the bridge
box(715, 625, 120, 50, "OLED display", ["SSD1315, 128 x 64"], tsize=12)
line([(WX + WW, 650), (715, 650)])
text(630, 644, "I2C1", size=10, anchor="middle", fill=C_SIG)
box(715, 692, 120, 64, "Power button", ["press: on", "hold 3 s: off"], tsize=12)
line([(WX + WW, 724), (715, 724)])
text(630, 718, "PB5 sense", size=10, anchor="middle", fill=C_SIG)

# compute link
LINKX = 455
line([(LINKX, WY), (LINKX, CMY + CMH)], color=C_LINK, width=2.6)
text(LINKX - 6, 410, "UART link to the bridge", anchor="end", fill=C_LINK)
text(LINKX - 6, 424, "USART1 to UART7, 3.3 V, 2 Mbps", size=10, anchor="end", fill=C_LINK)

# +BATT rail on the right: up to the head buck, down to the servos
RAIL = 850
line([(685, 573), (RAIL, 573)], color=C_BATT)
dot(RAIL, 573, C_BATT)
line([(RAIL, 573), (RAIL, 384), (OX + OW + 2, 384)], color=C_BATT, arrow="d1453b")
text(RAIL - 6, 460, "+BATT, head bundle", size=10, anchor="end", fill=C_BATT_T)
line([(RAIL, 573), (RAIL, 862)], color=C_BATT, arrow="d1453b")
text(RAIL - 6, 822, "+BATT to segments", size=10, anchor="end", fill=C_BATT_T)

# ---------------------------------------------------------------- servos
SEGS = [
    ("Segment A", "left leg, USART2", 375, 30, 808,
     [(2, "left_hip_pitch"), (3, "left_knee"), (4, "left_ankle")]),
    ("Segment B", "right leg, USART3", 410, 235, 818,
     [(12, "right_hip_pitch"), (13, "right_knee"), (14, "right_ankle")]),
    ("Segment C", "hip yaw and roll, LPUART1", 440, 440, None,
     [(0, "left_hip_yaw"), (1, "left_hip_roll"), (10, "right_hip_yaw"), (11, "right_hip_roll")]),
    ("Segment D", "neck and head, UART4", 510, 645, 808,
     [(5, "neck_pitch"), (6, "head_pitch"), (7, "head_yaw"), (8, "head_roll"), (9, "mouth")]),
]
SY0, PITCH, SH, SW = 920, 46, 38, 150
for name, sub, pin, bus, turn_y, servos in SEGS:
    tx = bus + 14
    text(tx, 896, name, size=12, bold=True, fill=C_BUS)
    text(tx, 910, sub, fill=C_BUS)
    last_mid = SY0 + PITCH * (len(servos) - 1) + SH / 2
    if turn_y is None:
        pts = [(pin, WY + WH), (bus, last_mid)]
    else:
        pts = [(pin, WY + WH), (pin, turn_y), (bus, turn_y), (bus, last_mid)]
    line(pts, color=C_BUS, width=2.6)
    dot(pin, WY + WH, C_BUS)
    for i, (sid, joint) in enumerate(servos):
        y = SY0 + PITCH * i
        mid = y + SH / 2
        rect(tx, y, SW, SH, stroke=C_BUS)
        text(tx + SW / 2, y + 16, f"J288  ID {sid}", size=12, anchor="middle", bold=True,
             fill=C_TEXT)
        text(tx + SW / 2, y + 30, joint, anchor="middle")
        line([(bus, mid), (tx, mid)], color=C_BUS, width=1.8)
        dot(bus, mid, C_BUS)

# ---------------------------------------------------------------- legend and notes
LY = 1180
items = [(C_BATT, "+BATT, 21 to 25.2 V"), (C_5V, "5 V"), (C_BUS, "J288 servo bus, 6 Mbps"),
         (C_LINK, "compute link"), (C_SIG, "sensor, audio, IO")]
x = 20
for color, label in items:
    line([(x, LY), (x + 28, LY)], color=color, width=3)
    text(x + 36, LY + 4, label, size=12, fill=C_TEXT)
    x += 36 + len(label) * 6.6 + 24
rect(x, LY - 8, 28, 16, fill="#f3f8ef", stroke="#6f9a5a", rx=3, dash="6 3", sw=1.4)
text(x + 36, LY + 4, "PCB", size=12, fill=C_TEXT)

notes = [
    "Servo IDs are the joint's index in the firmware's JOINT_NAMES (motor-setup.md).",
    "Each segment connector on the bridge carries SIGNAL, +BATT through its own fuse, and GND.",
    "Segment D and the head bundle run up the neck harness: a 3-pin PH 2.0 for the segment, and a",
    "4-pin PH 2.0 (+BATT, GND, TX, RX) for the head buck and the link. All grounds are common.",
    "Details in docs/hardware.md, docs/bridge-pcb.md and docs/head-pcb.md.",
]
for i, s in enumerate(notes):
    text(20, 1208 + 15 * i, s)

out.append("</svg>")
SVG.write_text("\n".join(out) + "\n")
print(f"wrote {SVG}")
