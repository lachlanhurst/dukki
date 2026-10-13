# Head carrier PCB

Design note for a head PCB that carries the Radxa CM4 directly, in place of the Waveshare CM4-NANO-A and the loose head breakouts: the 6S to 5 V head buck, the 3.3 V regulator, the MAX98357A speaker amplifier, the fan MOSFET, the BMI088 IMU, the camera connector, and JST PH connectors for everything that stays off the board. It replaces the NANO-A, the head buck module and the point-to-point wiring of `hardware.md` sections 4.3, 4.6, 7, 9, 10 and 11.3 in the head. The NANO-A stays on the bench as the board the module is flashed on (`compute-setup.md`).

Date: 14/10/2026. Status: schematic done (section 7.1), ERC clean; layout revised for the on-board BMI088, the fallbacks and the 1.1 MHz buck (section 9.3), fully routed, DRC clean with schematic parity (0 violations, 0 unconnected), JLC package regenerated. Not yet reviewed in the KiCad GUI or ordered.

## 1. Requirements

From the decisions so far (12/10/2026):

- Board 80 mm wide by 55 mm tall, an M2 mounting hole in each corner. Two layers (section 8).
- The module sits at the left end, its 55 mm side along the board's 55 mm edge, about 6 mm in from the left edge so the two left-hand mounting holes clear it. Everything else goes in the free block to its right, about 34 x 55 mm.
- Built by JLCPCB PCBA, SMT parts on the top side only, all of them outside the module's footprint. Tracks may run under the module. Reuse the bridge's parts where possible, Extended ones above all (`bridge-pcb.md` section 6). The two boards are ordered separately for now; panelling them is a later option.
- On the board: the head buck (about 3 A at 5 V from the pack), the MAX98357A amplifier, the fan MOSFET, and the BMI088 (the bare chip since 13/10/2026; it was first a hand-fitted module).
- Fallbacks like the bridge's (13/10/2026): a link to cut so 5 V can come from an external step-down instead of the buck, and the same for the IMU and the amplifier, so external modules can stand in for them (section 6.1).
- The BMI088 was first the generic module of `hardware.md` 6.2 (pictures in `docs/datasheets/bmi088-module-*.png`, 15 x 20 mm, rows 11 mm apart). It is now the bare chip, which JLC stocks (C194919); the module can still be used externally (section 6.1).
- Mounting in the head (12/10/2026): the board lies flat with its top face, the module side, facing down, and its bottom edge, the one with the camera connector, facing forward. Seen from above, the module is on the robot's left and the free block on its right.
- Camera: the 15-pin 1.0 mm FPC connector on the bottom edge, centred in the free block. The camera sits below the free block, on the same face as the connector, so the cable runs straight from J8 to the camera without folding round the board edge. Cable 50 to 100 mm, length not yet chosen.
- Off the board, on JST PH 2.0 vertical through-hole headers (B2B to B5B-PH-K-S), soldered by hand, not by JLC: ToF, microphone, eye LED, speaker, fan, the head bundle from the bridge, and a spare I2C8 breakout.
- I2C8 pull-ups: 4.7 kΩ on the board (R8, R9), fitted since the BMI088 module and its 3.3 kΩ went; the ToF breakout adds its 10 kΩ, about 3.2 kΩ together.
- Deferred: a maskrom button, the USB download port and the UART0 debug console. The module is flashed on the NANO-A.

## 2. Block diagram

```text
 J1 head bundle                                          Radxa CM4 (two DF40 sockets)
  1 +BATT --+-- TVS --+-- bulk C --+-- U1 buck 5 V --+==> 5V ==> pins 77 79 81 83 85 87
  2 GND     |         |            |                  |
  3 RX <----+---------+------------+------------------)----- UART7_TX  GPIO23  pin 47
  4 TX -------------------------------------------------->-- UART7_RX  GPIO24  pin 45
                                                      |
                                5V --+-- U2 AMS1117 --+--> 3V3: camera, ToF, IMU, mic, I2C breakout
                                     +-- U3 MAX98357A --> J5 speaker   (SAI2: pins 49, 26, 25)
                                     +-- Q1 fan FET -----> J6 fan      (PWM1_CH2: pin 50)
                                     +-- J4 eye LED                    (SPI1_MOSI: pin 44)
                                     +-- LED power

 3V3, I2C8 (pins 58, 56) --+-- JP4, JP5 -- U5 BMI088 (accelerometer 0x19, gyroscope 0x69)
                           +-- J2 ToF
                           +-- J3 I2C breakout
 3V3, SAI2 (pins 49, 26, 27) -- J7 microphone
 J8 camera FPC <-- CAM0 lanes (pins 128 to 142), I2C6 (pins 35, 36, 4.7 kΩ), CAM_GPIO (pin 97), 3V3
```

One ground node: the head bundle's ground is the return for the buck's input current and the reference for the bridge link, as `hardware.md` 11.3 relies on.

## 3. Power

### 3.1 Current budget

| Load | Rail | Peak | Note |
|---|---|---|---|
| Radxa CM4 | 5 V | 2.5 A | `hardware.md` section 10 budget |
| MAX98357A at full output | 5 V | 0.6 A | |
| Fan | 5 V | 0.2 A | 2-wire 5 V fan, PWM |
| Eye LED | 5 V | 0.06 A | one WS2812-type LED at full white |
| 3.3 V rail through the AMS1117 | 5 V | 0.4 A | camera about 0.25 A, ToF up to 0.1 A, IMU, mic |
| Total | 5 V | about 3.8 A | the peaks do not coincide; 3 A continuous |

From the pack that is about 0.85 A at 20 V and 90 % efficiency, which is why the bridge's head feed fuse F6 went to 2 A (`bridge-pcb.md` 3.2).

### 3.2 Input protection

The bundle runs about 30 cm up the neck beside servo power. At the input: an SMBJ30A TVS (C19077584, the bridge's part) and two 10 µF 50 V 1210 capacitors (C596319, the bridge's part) at the buck. The TVS clamps at up to about 48 V at its peak current, so the buck needs a rating of 60 V or more, not a 36 V part. No local fuse: F6 on the bridge protects the bundle and this board. The PH header is keyed, so no reverse-polarity part.

### 3.3 5 V buck

TI LMR51635YDDCR (C45262770, Extended, about 3,200 in stock): 4.3 to 60 V in, 3.5 A continuous, synchronous (no external diode), SOT-23-6, 1.1 MHz with light-load PFM. The same TI family as the bridge's LMR51606, one size up. The 400 kHz X version was first chosen for efficiency, but its stock fell to 76 by 13/10/2026; the Y version is the same chip switching faster, a few percent less efficient. Feedback 82k over 15k (both Basic) sets 5.17 V against the 0.8 V reference, a little over 5 V for the drop to the module's pins at 3 A; TI's own pair is 53.6k over 10.2k. The divider's top is on the buck's side of JP1 (section 6.1), so a cut link leaves the buck regulating its own capacitors instead of running open loop. Output three 47 µF 10 V X5R 1206 (C96123, Basic), about 66 µF after bias derating, against TI's 33 µF at 1.1 MHz. Input two 10 µF 50 V 1210, 2.2 µF and 100 nF; 100 nF bootstrap. EN tied to VIN. Note the pinout differs from the bridge's LMR51606: 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 CB. Inductor 4.7 µH (TI lists 3.3 µH for 5 V at 1.1 MHz; 4.7 µH gives about 0.8 A ripple at 25 V in): Sunlord SWPA6045S4R7MT (C78804, 5.5 A saturation, 6 x 6 x 4.5 mm), the bridge's inductor series.

Headroom: the high-side current limit is 4.2 A minimum. At 3.5 A out from 24 V the inductor peaks at about 4.0 A, so 3.5 A is the part's edge. Our continuous load is about 3 A and the peaks in 3.1 do not coincide. If the measured draw says otherwise, the step up is the TPS54560B (C1850354, 60 V, 5 A, HSOP-8, non-synchronous, needs a Schottky).

### 3.4 3.3 V

AMS1117-3.3 (C6186, Basic) from 5 V, as on the NANO-A. At 0.4 A it dissipates about 0.7 W, spread into a copper pour on the tab. The module's own 3.3 V output (pins 84 and 86) is used only for GPIO_VREF.

## 4. Module connections

Pi CM4 connector numbering: pins 1 to 100 on the first connector, 101 to 200 on the second. The Radxa CM4 keeps the Pi assignment on these two (`hardware.md` 4.1); its third connector is not fitted on this board. Pin numbers below are from the NANO-A schematic.

| CM4 pin | Pi name | RK3576 | Use on this board |
|---|---|---|---|
| 77, 79, 81, 83, 85, 87 | +5 V in | | buck output |
| 84 | +3.3 V out | | GPIO_VREF only. Pin 86 is the same rail, left open: ERC rejects two power outputs on one net |
| 78 | GPIO_VREF | | to the module's 3.3 V (pin 84), as the NANO-A in use has it (its jumper H2, confirmed 12/10/2026) |
| 58 | GPIO2 | I2C8_SDA_M1 | IMU, J2 ToF, J3 I2C breakout |
| 56 | GPIO3 | I2C8_SCL_M1 | IMU, J2 ToF, J3 I2C breakout |
| 47 | GPIO23 | UART7_TX_M0 | J1 pin 4, to the bridge's RX |
| 45 | GPIO24 | UART7_RX_M0 | J1 pin 3, from the bridge's TX |
| 50 | GPIO17 | PWM1_CH2_M1 | Q1 fan gate |
| 44 | GPIO10 | SPI1_MOSI_M0 | J4 eye LED data, through a series resistor |
| 49 | GPIO18 | SAI2_SCLK_M0 | U3 BCLK, J7 SCK |
| 26 | GPIO19 | SAI2_LRCK_M0 | U3 LRCLK, J7 WS |
| 25 | GPIO21 | SAI2_SDO_M0 | U3 DIN |
| 27 | GPIO20 | SAI2_SDI_M0 | J7 SD |
| 35 | ID_SC | I2C6_SCL_M3 | J8 SCL, 4.7 kΩ to 3.3 V |
| 36 | ID_SD | I2C6_SDA_M3 | J8 SDA, 4.7 kΩ to 3.3 V |
| 97 | Camera_GPIO | | J8 pin 11 (camera enable), as on the NANO-A |
| 128, 130 | CAM0_D0_N, _P | CSI1 | J8 pins 2, 3 |
| 134, 136 | CAM0_D1_N, _P | CSI1 | J8 pins 5, 6 |
| 140, 142 | CAM0_C_N, _P | CSI1 | J8 pins 8, 9 |
| every GND pin | GND | | ground plane |

Left unconnected, as on the NANO-A: nRPIBOOT (93; the NANO-A's BOOT switch grounds it for flashing), GLOBAL_EN (99), RUN_PG (92), nEXTRST (100), EEPROM_nWP (20), WiFi_nDisable (89) and BT_nDisable (91), whose 0 Ω links the NANO-A leaves unfitted. UART0 (pins 55, 51) stays unconnected until the debug console is wanted.

Not used: the I2S MCLK (GPIO16), the spare GPIOs, and the second connector's PCIe, USB, HDMI, DSI and CAM1 pins.

## 5. Connectors

JST PH 2.0, vertical through hole, hand soldered. Pin 1 is the square pad.

| Ref | Pins | Use | Pin order |
|---|---|---|---|
| J1 | 4 | head bundle from the bridge | 1 +BATT, 2 GND, 3 bridge TX to UART7 RX, 4 bridge RX from UART7 TX: the same order as the bridge's J6, so a straight 1:1 cable |
| J2 | 4 | ToF (VL53L8CX breakout, its Qwiic socket replaced with PH) | 1 GND, 2 3.3 V, 3 SDA, 4 SCL (Qwiic order) |
| J3 | 4 | spare I2C8 breakout | the same as J2 |
| J4 | 3 | eye LED | 1 5 V, 2 DATA, 3 GND |
| J5 | 2 | speaker | 1 OUT+, 2 OUT−. Bridge-tied: neither side is ground |
| J6 | 2 | fan | 1 5 V, 2 FAN− (switched by Q1) |
| J7 | 5 | INMP441 microphone | 1 3.3 V, 2 GND, 3 SCK, 4 WS, 5 SD. L/R tied to GND on the mic side |
| J8 | 15 | camera FPC, 1.0 mm, SMT | Pi camera order: 1 GND, 2 D0−, 3 D0+, 4 GND, 5 D1−, 6 D1+, 7 GND, 8 CLK−, 9 CLK+, 10 GND, 11 CAM_GPIO, 12 CAM_CLK (not connected), 13 SCL, 14 SDA, 15 3.3 V |

Vertical PH headers need no board edge: the plug enters square to the board, so they can stand anywhere in the free block.

The PH headers and the BMI088 module go on the top face with every other part (decided 12/10/2026), so in the head they point down, the same way as the module and the camera. Plugs enter from below; leave room under the free block for the cables beside the camera.

## 6. Circuits

- Fan: Q1 AO3400A (C20917, Basic), low side, gate from pin 50 through 100 Ω with 100 kΩ to ground so the fan stays off while the module boots; B5819W (C8598, Basic, the bridge's part) across the fan as the flyback diode. `expressd` runs the fan at full on exit, which is unchanged.
- Eye LED: data from pin 44 through 330 Ω to J4. The LED runs from 5 V and takes the 3.3 V data directly, as on the bench (`express-setup.md`). A level shifter is not fitted.
- Amplifier: MAX98357A with SD_MODE pulled to 5 V through 1 MΩ, which against the chip's internal 100 kΩ gives about 0.45 V and selects the (left + right) / 2 mono mix, as on the Adafruit breakout we tested. GAIN left open for 9 dB, with an unfitted resistor footprint to change it. 10 µF and 100 nF on VDD.
- IMU: BMI088 (U5) on I2C: VDD and VDDIO on 3.3 V with 100 nF each (C15, C16), PS and CSB1 high, CSB2 open (the datasheet allows it in I2C mode), SDO1 and SDO2 high, so the accelerometer is at 0x19 and the gyroscope at 0x69. The straps are chosen for the layout: the footprint keeps copper out from under the chip, and at 0.5 mm pitch there is no room for a via between two pins of another net, so the bottom pin row is all 3.3 V and the top row's grounds sit either side of the open CSB2. 0x69 is not Pollen's 0x68; the driver fork's detection tries 0x68 then 0x69 (`hardware.md` 6.2). INT1 to INT4 unconnected. It sits in the middle of the free block, away from the mounting holes and the buck.
- Camera: 4.7 kΩ pull-ups on I2C6 as on the NANO-A, 100 nF at J8's 3.3 V pin.
- Power LED: KT-0603G (C12624, the bridge's part) on 5 V.

## 6.1 Fallbacks

Like the bridge's backup 5 V (`bridge-pcb.md` 3.6), each of the three parts most likely to fail or to need changing can be bypassed with a knife and a few wires, without reworking the board:

- 5 V: JP1, a wide bridged solder jumper (2 x 3 mm pads, 1.2 mm copper strip, for 3 A), joins the buck's output (net 5V_BUCK) to the 5 V rail. Cut it and feed 5 V from an external step-down through J12, four wire pads at 2.54 mm: 1 B+ (the pack, out to the step-down's input), 2 GND, 3 5V in, 4 GND. Silkscreen: CUT: EXT 5V.
- IMU: JP4 and JP5, bridged solder jumpers in SDA and SCL between I2C8 and the BMI088. Cut both to take the chip off the bus and plug an external BMI088 module into J3 (the spare I2C header: GND, 3.3 V, SDA, SCL), which then is the head IMU. Silkscreen: CUT: EXT IMU.
- Amplifier: JP3, the same wide jumper, feeds the MAX98357A's 5 V (net AMP_5V). Cut it and wire a generic MAX98357A module to J11, seven wire pads in the module's own order: LRC, BCLK, DIN, GAIN (open), SD (open: the module has its own 1 MΩ pull-up), GND, Vin. The speaker then goes to the module's own outputs. Silkscreen: CUT: AMP. With JP3 cut the on-board chip is unpowered on the SAI2 lines; if it ever loads them, lift it too.

## 7. Parts

| Part | Use | LCSC | Class | Shared with the bridge |
|---|---|---|---|---|
| DF40C-100DS-0.4V(51), 1.5 mm stack | module sockets, two | C597931 | Extended | no |
| TI LMR51635YDDCR, 60 V 3.5 A sync buck, 1.1 MHz | U2 | C45262770 | Extended | no (same family) |
| Sunlord SWPA6045S4R7MT, 4.7 µH 5.5 A | L1 | C78804 | Extended | no (same series) |
| Bosch BMI088, LGA-16 3 x 4.5 | U5 | C194919 | Extended | no |
| XUNPU FPC-1.0HF-15PWAR-H20, 15-pin 1.0 mm, dual contact, flip lock | J8 | C19269369 | Extended | no |
| MAX98357AETE+T, TQFN-16 3 x 3 | U3 | C910544 | Extended | no |
| AMS1117-3.3 | U2 | C6186 | Basic | |
| AO3400A | Q1 | C20917 | Basic | |
| B5819W | fan flyback | C8598 | Basic | yes |
| SMBJ30A | input TVS | C19077584 | Preferred | yes |
| 10 µF 50 V X7R 1210 | buck input | C596319 | Extended | yes |
| KT-0603G | power LED | C12624 | Extended | yes |

Basic parts besides: 0603 resistors (100R, 330R, 1k, 4.7k, 15k, 82k, 100k, 1M), 100 nF 50 V 0603, 2.2 µF 50 V 0805, 10 µF and 22 µF 25 V 0805, 47 µF 10 V 1206. Every LCSC number is in the schematic.

Not in the BOM: the PH headers, the module itself, the mounting hardware; the solder jumpers are copper only.

Eight Extended types, about AUD 4.30 each in feeder fees (about AUD 34 an order): six only this board uses (the socket, buck, inductor, FPC, amplifier and BMI088) and two the bridge also uses (the 10 µF capacitor and the LED), whose fees are paid once only if the two boards are ever ordered together. The MAX98357A's 9-ball WLP (C2682619) is cheaper but a 1.3 mm BGA, not worth it on two layers.

Camera cable: the standard Pi 15-pin camera cable has its contacts on opposite faces at the two ends. A dual-contact connector takes either face, so J8 works whichever way the cable folds under the board; check the pin 1 end with a cable before soldering a camera. A same-face cable reverses the pins and can damage the camera.

Socket footprint: KiCad's library has no DF40 footprint. Raspberry Pi's CM4 IO board KiCad files (`CM4IO-KiCAD.zip`, datasheets.raspberrypi.com) carry one footprint, `Raspberry-Pi-4-Compute-Module`, with both sockets and the four holes in their exact places. Use it rather than drawing the sockets.

## 7.1 Schematic

`hardware/head/head.kicad_sch` (KiCad 10), PDF beside it. Bootstrapped by `scripts/kicad/gen_head_sch.py` in the same way as the bridge's: every pin connects through a net label, power symbol or no-connect flag, and from now on the schematic file is the source (the script refuses to overwrite it without `--force`). Library: `hardware/kicad/DukkiHead.*`, made by `scripts/kicad/gen_head_kicad.py` (`hardware/kicad/README.md`). ERC: no violations. The netlist was checked net by net against sections 4 to 6.

References: J1 head bundle, J2 ToF, J3 I2C breakout, J4 eye LED, J5 speaker, J6 fan, J7 microphone, J8 camera, J9 and J10 the module sockets (pins 1 to 100 and 101 to 200), H1 to H4 the corner holes, H5 the module (outline, keep-out and its M2.5 holes), U2 buck, U3 AMS1117, U4 MAX98357A, U5 BMI088, Q1 fan FET, JP1 the 5 V link, JP3 the amplifier's link, JP4 and JP5 the IMU's, J11 and J12 the fallback wire pads. The two sockets are separate parts so JLC places both; the layout must keep them at their exact offsets from H5 (`hardware/kicad/README.md`).

## 8. Layers and the camera lanes

Two layers. Signals on the top, the bottom kept as an unbroken ground plane except for the through-hole pins of the PH headers and short jumps where a track must cross another.

- Camera lanes: two data pairs and the clock pair, 100 Ω differential, routed on the top as ground-guarded pairs over the bottom plane, using JLC's 2-layer impedance figures for the track width and gap. Length-matched within each pair, no vias, about 35 to 40 mm from the module's CAM0 pins (on the right-hand socket, about halfway down) to J8. The IMX219 runs below 1 Gbps a lane in the full-frame mode we use, and the flex cable to the camera is no better controlled than this. If the lanes misbehave, the camera's CRC error count shows it (`hardware.md` 5.5 records how it appeared before); the fallbacks are the binned sensor mode at half the lane rate, or a 4-layer version of this board alone.
- Module sockets: about 25 of the 200 pins are used. The inner row escapes inward under the module on the top layer and drops to vias there; the area under the module carries tracks but no parts. Everything on the left-hand socket (5 V in, I2C, UART7, SAI2, SPI1, the fan PWM, I2C6) crosses under the module to the free block, about 35 mm: 5 V as a pour of 2 mm or more on the top, the signals beside it.
- Buck: input capacitors, IC and inductor in a tight loop on the top, with the bottom plane under them unbroken.

## 9. Floor plan

Board coordinates in mm from the top-left corner. Placement inside the free block is still to be done.

```text
  0 +--------------------------------------------------------------------------+
    |(o)  +-----------------------------------+                             (o)|
    |     |                                   |                                |
    |     |  Radxa CM4, 40 x 55               |  free block, about 34 x 55     |
    |     |  pins 1-100 socket |  | pins 101- |  buck, AMS1117, MAX98357A,     |
    |     |  near left edge    |  | 200 near  |  fan FET, BMI088 module,       |
    |     |  (5 V, GPIO)       |  | right edge|  PH headers J1 to J7           |
    |     |                                   |                                |
    |(o)  +-----------------------------------+        [ J8 camera FPC ]    (o)|
 55 +--------------------------------------------------------------------------+
    0     6                                   46                              80
```

- (o) are the four M2 mounting holes. Their centres about 3 mm from each edge leave the screw heads about 1 mm clear of the module.
- The module's own four M2.5 holes, 3.5 mm in from its corners (x 9.5 and 42.5, y 3.5 and 51.5), with spacers to suit the 1.5 mm stack.
- J8 centred in the free block on the bottom edge, about x = 63. It is 19 mm long and about 6 mm deep with the latch open.
- Module orientation, from the CM4 datasheet's top view (module x along 55 mm, y along 40 mm, origin bottom-left, pins 1 to 100 along the top edge, 101 to 200 along the bottom, both sockets centred at x = 25, pin 1 and pin 101 at x = 34.8): turned 90° anticlockwise, so pins 1 to 100 run down the left and 101 to 200 down the right, next to the free block. In board coordinates a module point (x, y) lands at (46 − y, 55 − x). The CAM0 pins (128 to 142) then sit at about y = 25 to 28, the 5 V pins (77 to 87) at about y = 35 to 37 on the far left, and the Wi-Fi socket in the module's top-left corner ends up at the board's bottom-left.
- The antenna lead leaves from the board's bottom-left corner; check it reaches the antenna mount on the shell top.

## 9.1 Layout (13/10/2026)

`hardware/head/head.kicad_pcb`, made by `scripts/kicad/gen_head_pcb.py` (placement, from the netlist) and `scripts/kicad/route_head_pcb.py` (hand-drawn copper, Freerouting, pours, stitching), both one-shot bootstraps like the bridge's. Three rounds so far; what each taught is kept here because it shapes the next.

Placement, as it stands:

- The module (H5) is turned 180 degrees from the floor plan in section 9: J9 (pins 1 to 100: 5 V, GPIO, SAI2, UART7, I2C) runs down its right-hand side, against the free block, and J10 (101 to 200, the camera lanes) down its left. With J9 on the far side, its 5 V pins and nearly every signal had to cross the module and squeeze round the socket's ends; turned, they open straight into the free block, 5 V is a 15 mm run from the buck, and only the camera pairs cross the module. H5, J9 and J10 are locked at their fixed offsets. The module's Wi-Fi U.FL now sits at its top-right corner, by the buck.
- Left strip (5.7 mm, above the camera pairs): ToF (J2) and eye LED (J4). Right edge, top to bottom: head bundle (J1), microphone (J7), speaker (J5), fan (J6). The spare I2C header (J3) stands in the free block beside the BMI088.
- Free block: buck top right by the head bundle, power LED in the top-left corner, AMS1117 and amplifier below the buck, fan FET by the head bundle, BMI088 module above the camera connector, camera I2C parts left of J8. J8 is turned so its contact pads, the cable entry side on XUNPU's drawing, face the bottom edge. No courtyard overlaps.
- Connector names and pin 1 marks are on the back silkscreen, which faces up in the head.

Hand-drawn and locked:

- Camera pairs: 0.2 mm tracks at the socket's 0.4 mm pitch, about 85 mm, from J10 west into the left strip, south, east along the module's lower edge and under the BMI088, and south into J8. N stays on the left of the direction of travel, so it lands east of P as J8 needs, and the pairs nest without crossing. Ground pour either side with a stitching via row about 1 mm out; the bottom plane under them is kept free of tracks by rule areas, except a 4.5 mm gap at x 51.5 to 56 where the camera's control lines and 3.3 V must cross to J8.
- 5 V: L1 and the output capacitors to a locked copper area over J9's six 5 V pads, with a branch to the AMS1117's input. The eye LED's 5 V and data cross under the module on the bottom layer to the left strip.
- Rejected on the way: a 5 V run down the left of J9 on the top layer (it walled in the socket's outer row) and a diagonal one under the module on the bottom (it cut the bottom layer in two).

Routing, round three (13/10/2026):

- UART7 and the SAI2 bit clock leave J9's outer row through vias beside it and cross the free block on the bottom layer, under the parts that wall in the top layer. RX and TX swap order between J9 and J1, so TX comes back to the top for its last few millimetres; SCLK comes up near the microphone header and has a branch up to the amplifier's BCLK. That took Freerouting from 12 open connections to 7.
- `scripts/kicad/finish_head_pcb.py` then runs Freerouting once more over the routed board, keeping what is there, with signal clearance at 0.13 mm (JLC's 2-layer minimum is 0.127 mm), and refills the pours. It routed the last 7.
- DRC with schematic parity: no errors, no warnings. Two fixes on the way: the hand-fitted parts and the unfitted resistors now carry the schematic's exclude-from-BOM and DNP flags (the netlist parser in `gen_head_pcb.py` matched the wrong syntax), and J8's ground pins take the pour solidly, as there is room for only one spoke between the camera pairs.
- 706 track segments, 645 vias (most of them ground stitching), about 1.4 m of track on the top and 0.8 m on the bottom.

## 9.2 Fabrication package

`python3 scripts/kicad/export_jlc.py head` (the bridge's script, now taking the board name) writes `hardware/head/fab/`: `head-gerbers.zip`, `head-bom.csv` (26 lines) and `head-cpl.csv` (39 placements; regenerated 14/10/2026), same formats and origin as the bridge's. Layout PDF beside the board, `hardware/head/head-layout.pdf`, one page per layer as for the bridge: top copper, bottom copper, front silkscreen, back silkscreen (mirrored, so it reads as seen from the back, the side that faces up in the head), front fab with references, and the outline. 3D renders: `hardware/head/head-3d-top.png` and `-bottom.png` (no models for the sockets, FPC and BMI088 module).

Cost: JLC's parts API timed out on 13/10/2026, so `jlc_cost.py` was not run. Fees from the part classes: seven Extended types at about AUD 4.30 each, about AUD 30, plus setup and stencil as for the bridge.

Before ordering:

- Review the board in the KiCad GUI, in particular what the autorouter did around J9, the amplifier and the right-hand headers, and the 0.13 mm clearances.
- Check every part's rotation in JLC's placement preview, above all J8 (turned 180 degrees), the two sockets, the TQFN amplifier, the SOT-223 and the SOT-23-6 buck. JLC's footprint origins and rotations do not always match KiCad's.
- Print the socket and hole layout 1:1 and lay a module on it (section 10).
- Camera pairs: check 0.2 mm track and 0.2 mm gap against JLC's impedance calculator for 2-layer 1.6 mm coplanar with ground; widen or tighten the pair if it is far from 100 ohms.

## 9.3 Revision: BMI088 on the board, fallbacks, 1.1 MHz buck (13/10/2026)

- The BMI088 module's place went to the chip (U5) with C15, C16, the links JP4 and JP5 and the pull-ups R8 and R9, and to the two fallback wire-pad rows, J12 (external 5 V) above J11 (external amplifier), along the bottom of the free block. JP1 sits in the 5 V run west of the buck, JP3 below the amplifier.
- Hand-drawn additions in `route_head_pcb.py`: the buck's switch node and bootstrap (Freerouting once left C5 off the switch node), the 5 V through JP1, and the BMI088's straps (section 6). Freerouting hands locked vias back on import as second copies at the net class's size; both scripts now drop them.
- Silkscreen: pin names under J12 (B+ G 5V G) and J11 (LR BC DI GA SD G 5V), EXT 5V and EXT AMP over them, CUT: EXT 5V, CUT: AMP and CUT: EXT IMU beside the links, CAM by J8; connector names on the back beside each header (J3 now I2C/IMU); the name block (Dukki head, rev A 2026, Lachlan Hurst) centred on the back over the module.
- Module clearance, checked from the placed footprints: the module's outline runs x 5.95 to 46.05 and covers the board's full height. Nothing but the two sockets has a courtyard under it; tracks, tented vias and the camera pairs run beneath its 1.5 mm stack. Left of it, the ToF and eye LED headers (6 mm tall, beside the module rather than under it) ended 0.34 mm from its edge as first placed; they moved 0.3 mm out, leaving about 0.65 mm, with the housings 0.3 mm in from the board edge. On the right the nearest parts (JP1, the power LED, J12 and J11) start 0.55 mm out and are all lower than the module's underside. The corner screw heads (3.8 mm) end about 1 mm short of the module. The module's own M2.5 holes have nothing within 2.25 mm on the top except pour and the camera pairs, which pass 1.25 mm from a spacer's edge. A heatsink or fan wider than the module would meet the 6 mm headers on the left: keep it within the module's outline.
- The last connection (the I2C8 SCL pull-up R9 and JP5's bus side, stranded by two Freerouting runs) was made on 14/10/2026 by `scripts/kicad/route_one_net.py`, a small maze router: a 0.1 mm grid over both layers, every other net's copper grown by 0.2 mm plus half the track or via width as an obstacle, keep-outs respected, vias allowed but costly. It found a path with six vias, more than a hand route would take but DRC clean. Moving R8 and R9 beside the links and hand-drawing the bus side to J3 was tried first and left seven connections open after rerouting; that change is kept in a git stash, not on the board.

Tidy (14/10/2026, `scripts/kicad/tidy_head_pcb.py`, run after the routing): Freerouting left small jogs and kinks, and necked tracks down at tight spots; the maze router left a staircase; and the 610 stitching vias, placed after the main autoroute but present for the finishing passes and the maze router, hemmed those in. The tidy removes the stitching, then, wherever two unlocked segments of one net meet at a bare joint, replaces them with one straight segment if it clears every other net's copper by the larger of the two net classes' clearances and the keep-outs (306 joints over four sweeps), widens narrow segments to their net class where they fit, and stitches again more sparingly: a 3.5 mm grid and rows beside the camera pairs, only where both pours are filled (163 vias). DRC with schematic parity stays at 0 violations, 0 unconnected. Tracks are now straight between bends, at any angle. A second look (14/10/2026) found copper crossing ground pads' thermal-relief gaps: Freerouting's GND tracks, routed before any pour existed. All 127 track-on-pad overlaps were ground on ground, so nothing was wrongly connected, but the 215 unlocked GND segments only duplicated the pours; the tidy's `unground` stage removes them. DRC confirms every ground pad still reaches the pour; C9's ground pad needed a solid connection to do so. The six GND tracks left on pads are the hand-drawn, locked ones (the BMI088's ground pins to their vias). Segments still below their class width are ground stubs into the pour (0.2 to 0.45 mm against the class's 0.6 mm, which only matters for ground run as track), the hand-drawn BMI088 straps (0.2 mm at its 0.5 mm pitch), the 5 V into the module's pads and the switch-node branch to C5 (0.5 mm), and two 0.25 mm AMP_5V segments with no room to widen; every signal net is at its class width.

Cost (14/10/2026, JLC live prices, 5 boards): parts US$76.30, fees US$34.12 (setup 8, stencil 1.5, 8 Extended types 24, joints about 0.6), about US$110 or AUD 159 for assembly, plus bare boards, shipping and GST. `jlc_cost.py` counts the two socket lines as two Extended types (they share C597931, so JLC charges once) and two joints a part (the sockets have 100 each), so its own total reads US$3 high. The BMI088 is the dearest part, about US$6.56 each.

## 10. Open items

- The CM4 socket positions and mounting holes from Raspberry Pi's CM4 IO board design, confirmed with a printout against a module before ordering.
- The camera cable: a standard opposite-face Pi cable, 50 to 100 mm, pin 1 checked before the first power-up.
- The BMI088's mounted orientation in the head: the board's position and rotation become the MJCF `head_imu` site (`hardware.md` 6.2). The head body's frame in the MJCF is not x-forward: the `head_camera` camera looks along the body's +z, so map the board's axes through that frame, not through the robot's forward, left and up.
- Height under the board for the camera, and over it for the module heatsink.
