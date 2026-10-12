# Bridge carrier PCB

Design note for a trunk PCB that carries the bridge: the WeAct STM32G474 core board, the LSM6DSV16X breakout, a 5 V supply from the 6S pack, a soft start on plug-in, and the power distribution to the servo segments and the head. It replaces the trunk buck, the distribution board and the point-to-point wiring of `hardware.md` sections 5.8, 10 and 11.2 with one board.

Date: 11/10/2026. Status: schematic done, first layout placed and routed (section 6.2), before review and fabrication outputs.

## 1. Requirements

From the decisions so far:

- Modules: the WeAct STM32G474 core board (QFN48) and the LSM6DSV16X breakout already on hand, soldered to the carrier on short sections of header pin with about a 1 mm gap underneath, so tracks and tented vias may run under them but no parts. On the bench build they sit on female headers instead.
- Battery in on a male XT30, the solder-cup cable connector (XT30U-M), not a PCB-mount part. It stands straight up from the board face, centred near the top edge: its two solder cups (2.2 mm diameter, 5.0 mm apart, measured) go through KiCad's stock XT30U-M footprint, 2.7 mm plated holes on 3.8 mm pads, soldered by hand from the back. Not fitted by JLCPCB.
- A soft start so plugging the pack in does not spark. No electronic switch: the robot is on whenever the pack is plugged in (`hardware.md` section 10).
- A 5 V buck from the pack for the WeAct board's VCC input.
- SMD fuses. No current sensing.
- Every other connector is JST PH 2.0, vertical through hole (B2B/B3B/B4B-PH-K-S), soldered by hand, not by JLC. The plug enters from above, perpendicular to the board:

| Ref | Pins | Use | Pin order |
|---|---|---|---|
| J2 to J5 (segments A to D) | 3 | servo segments A to D (D not used on this robot) | 1 SIGNAL, 2 +BATT, 3 GND, as on the J288 pigtail |
| J6 (head) | 4 | head bundle (`hardware.md` 11.3) | 1 head buck +BATT, 2 GND, 3 bridge TX (PA9) to head UART7 RX, 4 bridge RX (PA10) from head UART7 TX |
| J7 (display) | 4 | status display | 1 GND, 2 3.3 V, 3 SCL (PA15), 4 SDA (PB7), the module's own order |
| J8 (button) | 2 | power and shutdown button | 1 BTN_A, 2 BTN_B: the button joins them (section 6.5) |

- Servo +BATT and GND carried on wide pours on both layers, stitched with vias.
- Board 45 mm wide by 80 mm tall, with a mounting hole in each corner. Everything tall (connectors, XT30, any capacitor over about 3 mm) in the top 20 mm; the flat modules and low parts below.
- Built by JLCPCB PCBA, parts on the top side only, preferring Basic parts. Each Extended part type adds a feeder fee of about AUD 4.60 (US$3).

## 2. Block diagram

```text
 XT30 --+-- F1 main --+-- Q1 soft start (P-FET) --+== +BATT_SW pour (both layers) ==+
        |             |                            |                                 |
       GND           TVS                        bulk C                     +--F2--> J2  seg A (PB3)
                                                   |                       +--F3--> J3  seg B (PB9)
                                                   |                       +--F4--> J4  seg C (PA2)
                                                   |                       +--F5--> J5  seg D (PC10)
                                                   |                       +--F6--> J6  head, pin 1
                                                   |
                                   U1 buck 5 V ----+---> WeAct VCC (P1-1, P1-2)
                                                          WeAct 3V3 --> IMU, OLED
                                   +BATT_SW --100k--+--10k-- GND
                                                    +--> PA0 (pack voltage), 100 nF
```

One ground node throughout: the soft start switches the positive side, so the pack's negative, the servo ground, the head's ground and the bridge's ground stay one net, as `hardware.md` 11.3 relies on.

## 3. Power

### 3.1 Current budget

PH 2.0 contacts are rated about 2 A, so the five branches cap continuous current near 9 A, with leg peaks of 2 to 3 A for short periods (`hardware.md` 11.3). The main path (XT30, F1, Q1, pours) is designed for 10 A continuous. Fifteen servos at their 1.92 A limit together (29 A) cannot pass through the connectors and is not a design case.

### 3.2 Fuses

A main fuse and one per branch. A short in a leg harness, wired in PH crimp wire, would burn the wire long before a 10 A main fuse opened, so each branch gets its own.

| Fuse | Rating | Candidate |
|---|---|---|
| F1 main | 10 A, 1206, time lag, 32 V DC, 150 A breaking | BHFUSE BSMD1206C-2100T, C41367232, Extended |
| F2 to F5 segments | 4 A, 1206, time lag, 72 V DC | BHFUSE BSMD1206C-1400T, C41367227, Extended |
| F6 head buck feed | 1 A, 1206, time lag, 72 V DC | BHFUSE BSMD1206C-1100T, C41367221, Extended |

All three are BHFUSE's BSMD1206C time-lag series (12/10/2026). The Littelfuse 0458 parts first chosen turned out to be very fast-acting, which opens within milliseconds at two to three times rating; a leg of five servos can in principle draw about 9.6 A in a stall, so a time-lag part rides through peaks a fast one would not. F1's 150 A breaking capacity is below what a 6S pack can deliver into a dead short; every 1206 fuse is in that range, and a larger fuse body (2410, or a blade) would be the step up if it ever matters. 4 A sits above the PH contact's 2 A on purpose: the fuse protects the harness wire against a short, and the contact's rating is a thermal one that short peaks do not reach. Revisit both with the measured walking draw (open item in `hardware.md` section 13).

### 3.3 Soft start

A high-side P-channel MOSFET (Q1) whose gate is pulled down slowly, so the switched rail rises over about 30 ms instead of in microseconds. Inrush is then the downstream capacitance times the ramp rate, and the only capacitance on the XT30 side of Q1 is the TVS and the FET itself, so the plug barely arcs.

```text
 +BATT_IN ---+--------------+------ S  Q1  D ------+---- +BATT_SW
             |              |          G           |
             R2 470k     C1 2.2u       |     R3 1k + C2 47n (Miller)
             |              |          |           |
             +------+-------+----------+-----------+
                    |       |
               D1 15 V      R1 470k
              (Vgs clamp)   |
                           GND
```

- R1 and R2 set the on-state gate drive: half the pack, 10.5 to 12.6 V across a 6S discharge, inside the FET's ±20 V. D1 clamps transients.
- C2, gate to drain through R3 (1 k, which keeps C2 from loading the gate at high frequency), sets the ramp. During the Miller plateau about 38 µA leaves the gate node, so the drain rises at about 810 V/s: 25 V in about 31 ms. Inrush is about 0.8 A per mF of downstream capacitance.
- C1, gate to source, stops the FET switching on at the instant of plug-in. Without it, C2 would hold the gate at the drain's 0 V while the source jumped to 25 V. With 2.2 µF against 47 nF, the gate starts within 0.6 V of the source, below the threshold.
- Turn-on delay before the ramp is about 0.2 s, set by R1, R2 and C1.
- The FET dissipates the energy stored in the downstream capacitance during the ramp, ½CV², over 30 ms, plus whatever the servos draw as they wake. Q1's single-pulse SOA at 10 to 100 ms has to cover it. Measured 10/10/2026 with a meter's capacitance range across VCC and GND, unpowered: 64, 60 and 56 µF on three J288s, so about 60 µF each, ordinary electrolytic spread. Fifteen servos make about 0.9 mF, and the board's bulk (section 3.4) brings the worst case to about 1.2 mF. That gives about 1 A of inrush and about 0.4 J at 25 V, well under the 2 mF (0.6 J) first assumed here.

Chosen: NCE40P70K (C130101, Extended, 5,600 in stock), 40 V, 10 mΩ at 10 V, TO-252, pin 1 gate, tab drain, pin 3 source. KiCad's stock library has the TO-252 footprint and the larger package gives more pulse margin than the Diodes DMP4015SPSQ (C384564, 7 mΩ, PowerDI5060-8), which needs a footprint drawn from its datasheet; it stays the alternative if the band runs short of room. At 10 A Q1 dissipates about 1 W, which its drain pour has to spread; average current is far lower.

A TVS on the input (SMBJ30A class, 30 V standoff) protects Q1 from plug-in ringing and regeneration spikes; it cannot protect the servos, whose 25.2 V limit is too close to a full pack for any TVS.

### 3.4 Bulk capacitance

In the schematic: two 10 µF 50 V X7R 1210 MLCCs (C3, C4) and two 2.2 µF 50 V MLCCs (C5, C6). The polymer capacitors were dropped at layout: at 7.7 mm tall they needed the top band, which the connectors fill. The servos carry about 0.9 mF between them, so the board's own bulk was only ever a supplement.

After Q1, to absorb regeneration when servos brake and to stiffen the rail at the branch connectors. Two ways, depending on height:

- Low profile: four to six 10 µF 50 V X7R 1210 MLCCs (2.5 mm tall, so they can sit outside the tall band). At 25 V they keep about 3 to 4 µF each.
- Tall: one or two 35 V polymer capacitors, 100 µF in 6.3 x 7 mm (C2982822, 66,000 in stock) or 220 µF in 8 x 9 mm, inside the tall band.

The servos carry their own input capacitors and a LiPo absorbs regeneration well, so the board's bulk is a supplement. Start with the MLCCs plus one polymer if it fits.

### 3.5 5 V buck

TI LMR51606XDBVR (C21260865, Extended, about 9,700 in stock): 4 to 65 V in, 600 mA, SOT-23-6, 400 kHz, light-load efficient mode. A 65 V rating leaves margin over a full pack plus transients, where 36 V parts such as the TPS560430 sit too close. Load is the WeAct board, IMU and OLED, under 150 mA. Output to the WeAct's VCC pins (P1-1, P1-2) through D8 and JP1; the board's own LDO makes 3.3 V. Inductor and capacitors from TI's datasheet, all low profile. Feedback 130k/22k sets 5.53 V at the buck, about 5.2 V on the rail after D8.

USB. The WeAct board feeds its USB 5 V to VCC through its own Schottky (D4, B5819W), so USB can stay plugged in with the pack connected: the buck's 5.2 V holds that diode off and nothing flows into the computer, even if a fault put pack voltage on the 5 V rail. Without D8, USB alone would feed back through the buck: from its output through L1 and the high-side switch's body diode onto the switched rail at about 4 V, charging the servos' input capacitors (about 0.9 mF) and running fifteen J288s below their 6.4 V minimum from the USB port, with a few volts on the XT30's + pin. D8 (B5819W, C8598, Basic) between the buck's output and JP1 blocks that path. With both connected, the buck and USB share through their two diodes.

### 3.6 Backup 5 V

In case the on-board buck does not work, the board can take an external step-down instead:

- J9, four plated wire pads at 2.54 mm in the bottom-right corner, silkscreened B+ G 5V G: pin 1 is the switched rail (+BATT_SW, after the soft start, so the external module also gets the inrush limit), 2 ground, 3 the 5 V rail in, 4 ground.
- JP1, a solder jumper bridged by a copper trace, joins the buck's output (net 5V_BUCK: L1, C11, C12 and the feedback divider, then through D8) to the 5 V rail that feeds the WeAct board. Silkscreen beside it: CUT FOR EXT 5V. Cutting it isolates the buck, which then regulates only its own output capacitors.
- To use the backup: cut JP1, wire the external module's input to J9 pins 1 and 2 and its output to pins 3 and 4. The switched rail and the servos are unaffected.
- If the soft start itself ever fails, the servos lose power regardless of the 5 V; the fix then is a wire across Q1, from its source pad to its tab.

### 3.7 Pack voltage

100 k from +BATT_SW, 10 k to ground, 100 nF at PA0, as `hardware.md` 5.8 specifies. Full scale 36 V.

## 4. Signals

The pin allocation is `hardware.md` section 5.8, unchanged.

| Net | WeAct pin | To |
|---|---|---|
| Segment A, B, C, D data | PB3, PB9, PA2, PC10 | J2 to J5 pin 1, each through a 0 R link with room for a series resistor (hardware.md 5.6). The 5.6 V zener footprints first planned were dropped 12/10/2026 to make room for the soft power switch; they were never fitted and the bench runs without them |
| Link TX, RX | PA9, PA10 | J6 pins 3 and 4 |
| IMU SCK, MOSI, MISO, CS, INT1 | PB13, PB15, PB14, PA4, PA8 | breakout SCL, SDA, AD0, CS, INT1; VCC to 3V3 |
| OLED SCL, SDA | PA15, PB7 | J7 |
| Shutdown button | PB5 | J8 |
| Pack voltage | PA0 | divider |

The IMU breakout's three-pin row (SDX, SCX, INT2), measured 12.0 mm from the 7-pin row, gets plated holes but no connections: header pins there support the breakout against vibration (footprint in `hardware/kicad/`, see its README).

## 5. Floor plan

As laid out (11/10/2026), board coordinates in mm from the top-left corner:

```text
  0 +---------------------------------------------+  top edge
    | (o)  SEG A      HEAD      SEG B         (o) |  row 1: J2, J6, J3
    | SEG C OLED  BATT [ XT30 ]           SEG D   |  J4, J7, J1, J5: side connectors and the display turned along their edges
    |               BTN          F1  TVS          |  J8, main fuse, input TVS
 20 +---------------------------------------------+  end of the tall band
    | LED, divider,  gate     Q1        branch    |  low parts only
    | data-line      network  (TO-252)  fuses     |
    | R, power switch        bulk MLCC F2..F6     |
 36 +------------------------------------+--------+
    |                                    |        |
    |  WeAct G474 core board, horizontal | 5 V    |  1 mm stand-off
    |  36.3 x 28.1, USB-C at left edge   | buck   |
    |                                    | JP1    |
 64 +-------+--------------------+-------+--------+
    |       |  LSM6DSV16X        |                |
    | (o)   |  breakout, pins up |  J9 backup (o) |  between the lower holes
 80 +-------+--------------------+----------------+  bottom edge
```

- (o) are the four M2 mounting holes (2.2 mm), centres 3.5 mm from each edge.
- The head connector is top centre and the XT30 second-row centre (swapped 11/10/2026 for better head and battery cable runs).
- SEG C and SEG D are turned 90° outward, pin rows along the left and right edges with the housing's latch side facing the edge. The display connector (J7) is turned the same way as SEG C and stands beside it.
- Ground pins that carry current connect solid to both pours, not through thermal spokes: the XT30's ground pad, the ground pins of the five servo and head connectors, and the WeAct board's ground header pins. Spokes (0.5 mm, at most four a layer) would have choked the battery return, about 10 A through one pad. Solid pads take a hotter iron: 60 W or more and a large tip for the XT30.
- Locked 2.5 mm ground copper on both layers runs from the XT30's ground pad towards the + pad and up and down, so no signal track passes between or beside the pads, and stitching vias ring the pad.
- The WeAct board is horizontal, as the user asked, with its USB-C at the left board edge so a cable reaches it from the side. Its long side does not fit between the corner holes, so it sits between the top band and the IMU.
- The IMU breakout sits at the bottom, centred between the lower holes, turned 180° so its pin row faces the WeAct's P1 row (PB13 to PB15 and PA8 are there). In this orientation the chip's X axis points left and Y down the board; the mount quaternion must use that (hardware.md 6.1).
- The high-current path (XT30 +, F1, Q1, the switched-rail trunk and the fuse column) stays in the top band and the right of the middle band.

### 5.1 Fitting the tall band

The through-hole PH connectors (9.0 x 5.6 mm courtyard for 3 pins, 11.0 x 5.6 for 4) are far smaller than the SMD ones first chosen (11.4 x 8.5), which let the display and button return to connectors and still leaves room in the band for F1 and the TVS.

## 6. Parts so far

| Function | Part | LCSC | Library |
|---|---|---|---|
| 5 V buck | TI LMR51606XDBVR | C21260865 | Extended |
| Buck inductor | Sunlord SWPA4030S330MT, 33 µH, 4 x 4 x 3 mm | C83470 | Extended |
| Soft-start FET | NCE40P70K, TO-252 | C130101 | Extended |
| Main fuse | BHFUSE BSMD1206C-2100T, 10 A time lag | C41367232 | Extended |
| Segment fuses | BHFUSE BSMD1206C-1400T, 4 A time lag | C41367227 | Extended |
| Head feed fuse | BHFUSE BSMD1206C-1100T, 1 A time lag | C41367221 | Extended |
| Bulk | 10 µF 50 V X7R 1210 | C596319 | Extended |
| Input TVS | SMBJ30A | C19077584 | Preferred |
| Gate clamp | BZT52C15, SOD-123 | C19077412 | Preferred |
| Power LED | KT-0603G, green | C12624 | Extended |
| XT30, male, solder cup | Amass XT30U-M | hand fitted, cups through 2.7 mm plated holes | |
| PH connectors | JST B3B-PH-K-S (x4), B4B-PH-K-S (x2), B2B-PH-K-S | hand fitted | |

Resistors and the 100 nF, 47 nF, 2.2 µF and 22 µF capacitors are Basic parts; the schematic carries every LCSC number. That makes 10 Extended part types, about AUD 46 in feeder fees per order (JLC waives the fee for Preferred parts), of which the LED (dispensable) and the three fuse types are the easiest to trim. Genuine JST through-hole PH headers fit the servo pigtails' housings for certain; buy them with the XT30.

## 6.1 Schematic

`hardware/bridge/bridge.kicad_sch` (KiCad 10), PDF beside it. Bootstrapped by `scripts/kicad/gen_bridge_sch.py`, which every pin connects through a net label, power symbol or no-connect flag, so there are no wires; from now on the schematic file is the source and the script refuses to overwrite it. ERC: no violations. Project symbols (the LMR51606, from KiCad's LMR50410 with the same pinout, and a TO-252-numbered P-MOSFET) are in `hardware/kicad/Dukki.kicad_sym`.

Not in the BOM, fitted by hand or not parts at all: the XT30 (J1), all seven PH connectors (J2 to J8), the WeAct board (U2), the IMU breakout (U3) and the mounting holes.

## 6.2 Layout

`hardware/bridge/bridge.kicad_pcb`, 2 layers, 45 x 80 mm, 1 mm corner radius. Bootstrapped by two scripts run on the KiCad VM, after which the board file is the source:

- `scripts/kicad/gen_bridge_pcb.py` builds the board from the schematic netlist, the same links "Update PCB from Schematic" makes, places every part (the `PLACE` table holds the floor plan), draws the outline and puts the hand-assembly labels on the silkscreen. Reference designators are on the fab layer only: JLC places by the CPL file.
- `scripts/kicad/route_bridge_pcb.py` sets the net classes (segment feeds 1.2 mm, head feed and buck switch node 0.6 mm, other power 0.5 mm, signals 0.25 mm), draws and locks the high-current copper by hand on both layers, tied with via pairs (XT30 + to F1 and F1 to Q1's source at 3 mm each side, necking to 1.6 mm for the last 1.5 mm into the fuse's pads, which are only 2.9 mm apart; the switched-rail trunk at 2 mm on top plus its pour and 3 mm underneath; the fuse column at 2 mm on top plus the pour). By IPC-2221 one 3 mm track of 1 oz copper carries about 5 A for a 10 degree rise and 2 mm about 4 A, so a single layer would have run 40 to 50 degrees warm at the 10 A design current; two layers bring it to about 10 °C. 2 oz copper is the alternative if it is ever needed. The script also routes the four segment feeds by hand, each in its own lane (left to the autorouter, they knocked one another out once the power copper was locked; SEG C's runs round the outside of the board on the top layer, right edge, top edge, left edge, so it walls nothing off), then autoroutes the rest with Freerouting, pours ground on both layers and a solid switched-rail pour over the trunk, and stitches the ground pours with a via grid wherever clearance allows.

Freerouting 2.5.0 needs Java 25. Both live in `~/opt` on the VM (an Eclipse Temurin JRE and the release jar from GitHub, no system install). DRC on the routed board: no copper errors and nothing unconnected.

## 6.3 Fabrication package and cost

`scripts/kicad/export_jlc.py` (on the VM) writes `hardware/bridge/fab/`: `bridge-gerbers.zip` (copper, mask, paste, silkscreen, outline, separate PTH and NPTH Excellon files), `bridge-bom.csv` and `bridge-cpl.csv` in JLC's column formats. Only parts with an LCSC number, in the BOM and not DNP go to assembly.

Checked against JLC's 2-layer limits: narrowest track 0.187 mm (limit 0.127), clearance 0.2 mm (0.127), smallest via 0.6 mm on a 0.3 mm drill (ring 0.15, limit 0.13), smallest plated hole 0.75 mm, copper to edge 0.25 mm. Silkscreen text is 0.8 to 0.9 mm high against JLC's recommended 1.0 mm: legible, but small.

`scripts/kicad/jlc_cost.py` prices the BOM from JLC's live parts library (11/10/2026), with JLC's published Economic PCBA fees (US$8 setup, US$1.50 stencil, US$0.0016 a joint, US$3 per Extended part type, Preferred exempt), at A$1.44 per US$:

| Boards assembled | Parts | Fees | Assembly total |
|---|---|---|---|
| 5 | A$34 | A$49 | about A$83 |
| 2 | A$19 | A$49 | about A$68 |

Not included: the bare boards (2 layers, 45 x 80 mm, a few dollars for five), shipping to Australia and GST, which JLC's cart adds, and the hand-fitted parts (XT30U-M, seven JST PH-K headers). Of the fees, A$35 is the eight Extended part types (the buck, its inductor, Q1, three fuse values, the 10 µF bulk capacitors and the LED); dropping the LED or making the four segment fuses and the head fuse one value would each save A$4.30.

Before ordering, check in JLC's placement preview that the polarised parts sit the right way round: U1 (pin 1), Q1 (tab), D1 and D2 (cathode bar), D3 (LED cathode). KiCad and JLC disagree on the zero rotation of some packages.

## 6.4 3D model

`hardware/bridge/bridge.step` (about 12 MB, not committed: regenerate with `kicad-cli pcb export step --subst-models -o bridge.step bridge.kicad_pcb` in `hardware/bridge/` on the VM) holds the board and every part. KiCad's library supplies the SMD parts, the PH headers and the wire-pad header; `scripts/kicad/gen_3d_models.py` supplies the rest: the WeAct board from WeAct's own model, lifted to the 1 mm stand-off on short pins with its tall headers removed; the IMU breakout (outline, holes, chip, both pin rows, also at 1 mm); and an approximate XT30U-M (housing to the footprint outline, 11 mm tall, cups through the board), which KiCad's library lacks.

The board's drill and placement origin is its lower-left corner, and `export_jlc.py` writes the Gerbers, drill files and placement file relative to it. The first placement file measured from that corner while the Gerbers used KiCad's page coordinates; JLC flagged the offset and corrected it automatically.

## 6.5 Soft power switch and PA5 breakout (12/10/2026)

The soft start is now also an on/off switch (schematic block 9). Off means everything downstream of Q1 is off (servos, buck, bridge, LED, head feed), which answers the overnight drain in hardware.md section 10.

| Ref | Part | Job |
|---|---|---|
| Q2 | 2N7002 | grounds R1 (net PWR_LO): Q1 is on only while Q2 is on |
| JP2 | solder jumper, open | bridge it (silkscreen BRIDGE FOR ALWAYS ON) to ground PWR_LO permanently: the old behaviour, on whenever a pack is plugged in, soft start intact. It also keeps Q5 and Q4 off |
| R14, J8 | 470k, the button | the button joins BTN_A, fed from the pack through R14, to BTN_B. It switches about 30 µA; the contacts see pack voltage while open, which any tactile switch handles at that current, and R14 limits a pinched or shorted lead to microamps. J8's pins are BTN_A and BTN_B, no longer PB5 and ground |
| D9, R15, C14 | 12 V zener, 1M, 1 µF | clamp BTN_B to 12 V, and stretch a tap to about 1.3 s so it lasts until the bridge has booted and holds power (about 0.5 s) |
| D12 | 1N4148WS | BTN_B to Q2's gate (PWR_ON) |
| D11, R13 | BAT54WS, 1M | the bridge's hold: PC4 (PWR_HOLD) to PWR_ON, about 3 V; the diode keeps BTN_B's 12 V off PC4. R13 pulls the gate down, quickly, since the stretch capacitor sits on the button side |
| Q3 | 2N7002 | button sense: pulls PB5 low while BTN_B is high. PB5 sees only its pull-up and Q3, and a press reads low as on the breadboard, so one firmware build runs on both. Because of the stretch, PB5 stays low about 1.3 s after release |
| R20, D13, Q5 | 1M, 12 V zener, 2N7002 | with Q2 off, PWR_LO rises to Q1's gate voltage; Q5 turns on |
| Q4, R21, R22 | MMBT3906, 2.2M, 1M | Q5 turns Q4 on, pulling Q1's gate to its source in about 20 ms (C1 alone would take about a second, with Q1 half on meanwhile). Q4 also holds Q1 firmly off at plug-in |

Off-state leakage is about 20 µA (Q4's base current and R1, R20 into D13), decades for a pack, below its self-discharge. All parts are JLC Basic or Preferred: no new feeder fees.

PA5 breakout (J10): four plated wire pads at 2.54 mm in the bottom-left corner, silkscreened G 5V 3V3 A5. The board name block moved down below it.

Firmware rules (the same build on the breadboard, where PC4 is unconnected and the rules are no-ops):

- Assert the PC4 hold at boot only if the button is pressed then, so plugging in USB does not power the robot.
- Ignore a press already down at boot until it is released; only a fresh press can start the 3 s shutdown hold.
- After a committed shutdown, release PC4 once the head reports off and never assert it again or act on the button. If the bridge is still running a few seconds later, show "Release button" if it is held, otherwise "OFF, unplug pack" (the breadboard, or no power-hold hardware).
- A watchdog reset must not re-assert the hold unless the button is pressed.
- Low-voltage shutdown, firmware only once the switch exists: read the pack on PA0 (not read yet; the warning uses the servos' 0.5 V readings), and on the 10 s average below the threshold request the same shutdown the button does, so robotd sits the robot and halts the CM4, then release PC4. A lower hard floor (about 3.3 V a cell under load) cuts power after a short delay even if the head does not answer. On the breadboard the same build warns, requests the shutdown and shows "OFF, unplug pack".

PA5 breakout: four plated wire pads at 2.54 mm in the bottom-left corner (GND, 5V, 3V3, PA5), with the board name block moved down to make room.

## 7. Open items

- XT30 polarity: KiCad's XT30U-M footprint marks pad 1 (square) as − and pad 2 as +, and the schematic follows it (pin 1 GND, pin 2 +BATT). Check against the connector's moulded marks before soldering.
- Strain-relief holes beside the display and button pads, if wanted.
- Height limit outside the band: the WeAct's USB-C and buttons stand about 3.5 mm above the carrier plus the 1 mm stand-off, so anything under that should be fine.
- Review the routed board in the KiCad GUI before fabrication outputs: track routes Freerouting chose, the pours, and the silkscreen labels.
- Q1's SOA curve against about 0.4 J over 30 ms (section 3.3).
- Branch fuse ratings, from the measured walking draw.
- The WeAct board's underside: anything that stops it sitting flat.
