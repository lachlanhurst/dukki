#!/usr/bin/env python3
"""Make the 3D models the bridge carrier board needs beyond KiCad's library, as STEP files in
hardware/kicad/3d/. Needs CadQuery:

    uv run --python 3.12 --with cadquery python scripts/kicad/gen_3d_models.py [WeAct STEP]

Models are in footprint coordinates as KiCad expects (origin at the footprint origin, z = 0 on
the carrier's top surface, model +y = footprint -y), so the footprints use them at zero offset.

- WeAct_STM32G474_raised.step: WeAct's own model of the core board (fetch it as the README in
  hardware/kicad says; pass its path), with its tall pin headers removed, centred on the
  footprint and lifted so its underside sits 1 mm above the carrier, on short header pins
  through the carrier. Derived from WeAct's file, so not committed.
- LSM6DSV16X_Breakout.step: the generic breakout, outline and holes as measured from its photo,
  the IMU chip, raised 1 mm on header pins (7-pin row and the 3-pin support row).
- AMASS_XT30U-M.step: the male solder-cup XT30 standing on the board, approximate: housing to
  the footprint's outline, 11 mm tall, two contact openings, solder cups through the board.
  KiCad's library has no XT30U model.
"""
import sys
from pathlib import Path
import cadquery as cq

OUT = Path("hardware/kicad/3d")
OUT.mkdir(parents=True, exist_ok=True)
GAP = 1.0                      # module underside above the carrier, mm
CARRIER = 1.6

def pins(points, top, bottom=-(CARRIER + 1.4), size=0.64):
    """Square header pins at footprint (x, y) points, from below the carrier up to `top`."""
    w = cq.Workplane("XY")
    for x, y in points:
        w = w.add(cq.Workplane("XY").box(size, size, top - bottom, centered=(True, True, False))
                  .translate((x, -y, bottom)))
    return w.combine() if len(points) > 1 else w

# ------------------------------------------------------------------ WeAct core board, raised
if len(sys.argv) > 1:
    W, H, THICK = 36.28, 28.14, 1.59
    src = cq.importers.importStep(sys.argv[1])
    keep = [s for s in src.solids().vals()
            if not (s.BoundingBox().zmax > 8.0 and s.BoundingBox().zmin < -2.5)]   # drop the tall headers
    body = cq.Workplane("XY").newObject(keep).combine() if keep else None
    lift = GAP + THICK          # source board top is at z = 0, underside at -1.59
    body = cq.Workplane("XY").add(cq.Compound.makeCompound(keep)).translate((-W / 2, -H / 2, lift))
    x0, pitch = 6.97 - W / 2, 2.54
    rows = [H / 2 - 1.37, H / 2 - 1.37 - pitch]
    pts = [(x0 + i * pitch, sy * r) for i in range(12) for r in rows for sy in (1, -1)]
    asm = cq.Assembly()
    asm.add(body, name="weact", color=cq.Color(0.1, 0.1, 0.12))
    asm.add(pins(pts, GAP + THICK + 0.6), name="pins", color=cq.Color(0.85, 0.7, 0.2))
    asm.save(str(OUT / "WeAct_STM32G474_raised.step"))
    print("wrote", OUT / "WeAct_STM32G474_raised.step")

# ------------------------------------------------------------------ LSM6DSV16X breakout
BW, BH = 17.8, 15.0
Y_ROW = BH / 2 - 1.27
board = (cq.Workplane("XY").box(BW, BH, CARRIER, centered=(True, True, False))
         .translate((0, 0, GAP))
         .faces(">Z").workplane()
         .pushPoints([(6.8, -(Y_ROW - 11.5)), (-6.8, -(Y_ROW - 11.5))]).hole(2.2))
chip = cq.Workplane("XY").box(3.0, 2.5, 0.83, centered=(True, True, False)).translate((0.5, 0.5, GAP + CARRIER))
reg = cq.Workplane("XY").box(2.9, 1.6, 1.1, centered=(True, True, False)).translate((-5.6, -1.5, GAP + CARRIER))
row7 = [((i - 3) * 2.54, Y_ROW) for i in range(7)]
row3 = [((i - 1) * 2.54, Y_ROW - 12.0) for i in range(3)]
asm = cq.Assembly()
asm.add(board, name="pcb", color=cq.Color(0.05, 0.2, 0.6))
asm.add(chip, name="lsm6dsv16x", color=cq.Color(0.1, 0.1, 0.1))
asm.add(reg, name="regulator", color=cq.Color(0.1, 0.1, 0.1))
asm.add(pins(row7 + row3, GAP + CARRIER + 0.6), name="pins", color=cq.Color(0.85, 0.7, 0.2))
asm.save(str(OUT / "LSM6DSV16X_Breakout.step"))
print("wrote", OUT / "LSM6DSV16X_Breakout.step")

# ------------------------------------------------------------------ XT30U-M, approximate
HT = 11.0
outline = [(-2.6, -1.3), (-0.9, -2.6), (7.6, -2.6), (7.6, 2.6), (-0.9, 2.6), (-2.6, 1.3)]
housing = (cq.Workplane("XY").polyline(outline).close().extrude(HT)
           .faces(">Z").workplane().pushPoints([(0, 0), (5, 0)]).hole(2.6, 6.0))
contacts = (cq.Workplane("XY").pushPoints([(0, 0), (5, 0)]).circle(0.5).extrude(HT - 1.0))
cups = (cq.Workplane("XY").pushPoints([(0, 0), (5, 0)]).circle(1.1).extrude(3.4).translate((0, 0, -3.4)))
asm = cq.Assembly()
asm.add(housing, name="housing", color=cq.Color(0.95, 0.85, 0.1))
asm.add(contacts.union(cups), name="contacts", color=cq.Color(0.85, 0.7, 0.2))
asm.save(str(OUT / "AMASS_XT30U-M.step"))
print("wrote", OUT / "AMASS_XT30U-M.step")

# ------------------------------------------------------------------ WeAct core board, stand-in
# Ours, for board STEP files that go in the repo (export_step.py): the outline, the MCU, the USB-C
# receptacle and the short pins, at the same height as WeAct_STM32G474_raised.step.
W, H, THICK = 36.28, 28.14, 1.59
board = (cq.Workplane("XY").box(W, H, THICK, centered=(True, True, False))
         .edges("|Z").fillet(2.0).translate((0, 0, GAP)))
top = GAP + THICK
mcu = cq.Workplane("XY").box(7.0, 7.0, 0.9, centered=(True, True, False)).translate((2.0, 0, top))
usb = cq.Workplane("XY").box(7.4, 9.0, 3.2, centered=(True, True, False)).translate((-W / 2 + 3.0, 0, top))
x0, pitch = 6.97 - W / 2, 2.54
rows = [H / 2 - 1.37, H / 2 - 1.37 - pitch]
pts = [(x0 + i * pitch, sy * r) for i in range(12) for r in rows for sy in (1, -1)]
asm = cq.Assembly()
asm.add(board, name="pcb", color=cq.Color(0.1, 0.1, 0.12))
asm.add(mcu, name="stm32g474", color=cq.Color(0.05, 0.05, 0.05))
asm.add(usb, name="usb_c", color=cq.Color(0.75, 0.75, 0.78))
asm.add(pins(pts, top + 0.6), name="pins", color=cq.Color(0.85, 0.7, 0.2))
asm.save(str(OUT / "CoreBoard_STM32G474_standin.step"))
print("wrote", OUT / "CoreBoard_STM32G474_standin.step")

# ------------------------------------------------------------------ head carrier parts
# Footprints in hardware/kicad/DukkiHead.pretty that have no model of their own. Approximate:
# outlines and heights for clearance checks, not detailed parts.
STACK = 1.5                    # DF40C 1.5 mm mated height: CM4 underside above the carrier

# Radxa CM4: 40 x 55 mm, M2.5 holes 3.5 mm in from each corner, origin at the module centre.
# The parts on top are one envelope, 2 mm tall, as the module's own layout is not modelled.
CM4_T = 1.2
cm4 = (cq.Workplane("XY").box(40.0, 55.0, CM4_T, centered=(True, True, False))
       .edges("|Z").fillet(1.0).translate((0, 0, STACK))
       .faces(">Z").workplane()
       .pushPoints([(sx * 16.5, sy * 24.0) for sx in (1, -1) for sy in (1, -1)]).hole(2.7))
parts = (cq.Workplane("XY").box(30.0, 40.0, 2.0, centered=(True, True, False))
         .translate((0, 0, STACK + CM4_T)))
asm = cq.Assembly()
asm.add(cm4, name="pcb", color=cq.Color(0.05, 0.35, 0.15))
asm.add(parts, name="components_envelope", color=cq.Color(0.2, 0.2, 0.2))
asm.save(str(OUT / "Radxa_CM4_Module.step"))
print("wrote", OUT / "Radxa_CM4_Module.step")

# DF40C-100DS socket with the module's plug in it: the footprint's fab outline, full stack height
sock = cq.Workplane("XY").box(3.78, 22.8, STACK, centered=(True, True, False))
asm = cq.Assembly()
asm.add(sock, name="df40c_mated", color=cq.Color(0.9, 0.9, 0.85))
asm.save(str(OUT / "DF40C_Socket_CM4.step"))
print("wrote", OUT / "DF40C_Socket_CM4.step")

# XUNPU FPC-1.0HF-15PWAR-H20: 2.0 mm tall, contacts on the footprint's -y side (model +y),
# flip lock on the cable entry side
fpc = cq.Workplane("XY").box(17.8, 4.2, 1.6, centered=(True, True, False)).translate((0, -1.1, 0))
lock = cq.Workplane("XY").box(17.0, 1.4, 0.4, centered=(True, True, False)).translate((0, -2.5, 1.6))
asm = cq.Assembly()
asm.add(fpc, name="housing", color=cq.Color(0.95, 0.95, 0.9))
asm.add(lock, name="actuator", color=cq.Color(0.15, 0.15, 0.15))
asm.save(str(OUT / "FPC_XUNPU_1x15_P1.0mm_H2.0.step"))
print("wrote", OUT / "FPC_XUNPU_1x15_P1.0mm_H2.0.step")

# Generic BMI088 module, 15 x 20 mm, hand-fitted on short header pins like the bridge's modules.
# Footprint origin at pad 1; 7-pin row at y 0, 4-pin row 11.3 mm away offset 1.5 pitches.
mod = (cq.Workplane("XY").box(20.0, 15.0, CARRIER, centered=(True, True, False))
       .translate((7.62, 5.5, GAP)))
chip = cq.Workplane("XY").box(3.0, 4.5, 0.95, centered=(True, True, False)).translate((7.62, 5.5, GAP + CARRIER))
reg = cq.Workplane("XY").box(2.9, 1.6, 1.1, centered=(True, True, False)).translate((13.5, 8.5, GAP + CARRIER))
row7 = [(i * 2.54, 0.0) for i in range(7)]
row4 = [((i + 1.5) * 2.54, -11.3) for i in range(4)]       # footprint y, as pins() takes
asm = cq.Assembly()
asm.add(mod, name="pcb", color=cq.Color(0.05, 0.2, 0.6))
asm.add(chip, name="bmi088", color=cq.Color(0.1, 0.1, 0.1))
asm.add(reg, name="regulator", color=cq.Color(0.1, 0.1, 0.1))
asm.add(pins(row7 + row4, GAP + CARRIER + 0.6), name="pins", color=cq.Color(0.85, 0.7, 0.2))
asm.save(str(OUT / "BMI088_Module.step"))
print("wrote", OUT / "BMI088_Module.step")
