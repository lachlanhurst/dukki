# Head carrier PCB

Design note for a head PCB that carries the Radxa CM4 directly, in place of the Waveshare CM4-NANO-A and the loose head breakouts: the 6S to 5 V head buck, the 3.3 V regulator, the MAX98357A speaker amplifier, the fan MOSFET, the BMI088 module, the camera connector, and JST PH connectors for everything that stays off the board. It replaces the NANO-A, the head buck module and the point-to-point wiring of `hardware.md` sections 4.3, 4.6, 7, 9, 10 and 11.3 in the head. The NANO-A stays on the bench as the board the module is flashed on (`compute-setup.md`).

Date: 13/10/2026. Status: schematic done (section 7.1), ERC clean; first layout placed and partly routed (section 9.1).

## 1. Requirements

From the decisions so far (12/10/2026):

- Board 80 mm wide by 55 mm tall, an M2 mounting hole in each corner. Two layers (section 8).
- The module sits at the left end, its 55 mm side along the board's 55 mm edge, about 6 mm in from the left edge so the two left-hand mounting holes clear it. Everything else goes in the free block to its right, about 34 x 55 mm.
- Built by JLCPCB PCBA, SMT parts on the top side only, all of them outside the module's footprint. Tracks may run under the module. Reuse the bridge's parts where possible, Extended ones above all (`bridge-pcb.md` section 6). The two boards are ordered separately for now; panelling them is a later option.
- On the board: the head buck (about 3 A at 5 V from the pack), the MAX98357A amplifier, the fan MOSFET, and the BMI088 module.
- The BMI088 module is the generic board of `hardware.md` 6.2 (pictures in `docs/datasheets/bmi088-module-*.png`): 15 x 20 mm, a 7-pin row (VCC, GND, SCK, SDA, SDO, csA, csG) and a 4-pin row (INT4 to INT1) at 2.54 mm pitch, the two rows 11 mm apart (measured), two mounting holes at diagonal corners. Soldered on short header sections like the bridge's modules: tracks may run under it, parts may not.
- Mounting in the head (12/10/2026): the board lies flat with its top face, the module side, facing down, and its bottom edge, the one with the camera connector, facing forward. Seen from above, the module is on the robot's left and the free block on its right.
- Camera: the 15-pin 1.0 mm FPC connector on the bottom edge, centred in the free block. The camera sits below the free block, on the same face as the connector, so the cable runs straight from J8 to the camera without folding round the board edge. Cable 50 to 100 mm, length not yet chosen.
- Off the board, on JST PH 2.0 vertical through-hole headers (B2B to B5B-PH-K-S), soldered by hand, not by JLC: ToF, microphone, eye LED, speaker, fan, the head bundle from the bridge, and a spare I2C8 breakout.
- No I2C8 pull-ups fitted: the ToF breakout (10 kΩ through its TXS0104E) and the BMI088 module (3.3 kΩ network) carry their own, about 2.5 kΩ together. Footprints only, unfitted.
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

 3V3, I2C8 (pins 58, 56) --+-- U4 BMI088 module (SDO to GND: 0x18, 0x68)
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

TI LMR51635XDDCR (C42432812, Extended, about 3,400 in stock): 4.3 to 60 V in, 3.5 A continuous, synchronous (no external diode), SOT-23-6, 400 kHz with light-load PFM. The same TI family as the bridge's LMR51606, one size up. 400 kHz rather than the 1.1 MHz Y version for efficiency, since the heat stays in the head. Feedback 82k over 15k (both Basic) sets 5.17 V against the 0.8 V reference, a little over 5 V for the drop to the module's pins at 3 A; TI's own pair is 53.6k over 10.2k. Output three 47 µF 10 V X5R 1206 (C96123, Basic), about 66 µF after bias derating, for TI's 2 x 33 µF. Input two 10 µF 50 V 1210, 2.2 µF and 100 nF; 100 nF bootstrap. EN tied to VIN. Note the pinout differs from the bridge's LMR51606: 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 CB. Inductor 10 µH, TI's value for 5 V out at 400 kHz (datasheet SLUSF64B table 8-2): Sunlord MWSA0603S-100MT (C132141, 5.5 A saturation, 7 x 6.6 mm). No 10 µH part in the bridge's SWPA series has the saturation current, so the inductor is not shared.

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
- IMU: VCC to 3.3 V, SCK to SCL, SDA to SDA, SDO to GND (accelerometer 0x18, gyroscope 0x68), csA and csG unconnected. The INT row is soldered to pads with no net, for mechanical support only.
- Camera: 4.7 kΩ pull-ups on I2C6 as on the NANO-A, 100 nF at J8's 3.3 V pin.
- Power LED: KT-0603G (C12624, the bridge's part) on 5 V.

## 7. Parts

| Part | Use | LCSC | Class | Shared with the bridge |
|---|---|---|---|---|
| DF40C-100DS-0.4V(51), 1.5 mm stack | module sockets, two | C597931 | Extended | no |
| TI LMR51635XDDCR, 60 V 3.5 A sync buck | U1 | C42432812 | Extended | no (same family) |
| Sunlord MWSA0603S-100MT, 10 µH 5.5 A | L1 | C132141 | Extended | no |
| XUNPU FPC-1.0HF-15PWAR-H20, 15-pin 1.0 mm, dual contact, flip lock | J8 | C19269369 | Extended | no |
| MAX98357AETE+T, TQFN-16 3 x 3 | U3 | C910544 | Extended | no |
| AMS1117-3.3 | U2 | C6186 | Basic | |
| AO3400A | Q1 | C20917 | Basic | |
| B5819W | fan flyback | C8598 | Basic | yes |
| SMBJ30A | input TVS | C19077584 | Preferred | yes |
| 10 µF 50 V X7R 1210 | buck input | C596319 | Extended | yes |
| KT-0603G | power LED | C12624 | Extended | yes |

Basic parts besides: 0603 resistors (100R, 330R, 1k, 4.7k, 15k, 82k, 100k, 1M), 100 nF 50 V 0603, 2.2 µF 50 V 0805, 10 µF and 22 µF 25 V 0805, 47 µF 10 V 1206. Every LCSC number is in the schematic.

Not in the BOM: the PH headers, the BMI088 module and its header pins, the module itself, the mounting hardware.

Seven Extended types, about AUD 4.30 each in feeder fees (about AUD 30 an order): five only this board uses (the socket, buck, inductor, FPC and amplifier) and two the bridge also uses (the 10 µF capacitor and the LED), whose fees are paid once only if the two boards are ever ordered together. The MAX98357A's 9-ball WLP (C2682619) is cheaper but a 1.3 mm BGA, not worth it on two layers.

Camera cable: the standard Pi 15-pin camera cable has its contacts on opposite faces at the two ends. A dual-contact connector takes either face, so J8 works whichever way the cable folds under the board; check the pin 1 end with a cable before soldering a camera. A same-face cable reverses the pins and can damage the camera.

Socket footprint: KiCad's library has no DF40 footprint. Raspberry Pi's CM4 IO board KiCad files (`CM4IO-KiCAD.zip`, datasheets.raspberrypi.com) carry one footprint, `Raspberry-Pi-4-Compute-Module`, with both sockets and the four holes in their exact places. Use it rather than drawing the sockets.

## 7.1 Schematic

`hardware/head/head.kicad_sch` (KiCad 10), PDF beside it. Bootstrapped by `scripts/kicad/gen_head_sch.py` in the same way as the bridge's: every pin connects through a net label, power symbol or no-connect flag, and from now on the schematic file is the source (the script refuses to overwrite it without `--force`). Library: `hardware/kicad/DukkiHead.*`, made by `scripts/kicad/gen_head_kicad.py` (`hardware/kicad/README.md`). ERC: no violations. The netlist was checked net by net against sections 4 to 6.

References: J1 head bundle, J2 ToF, J3 I2C breakout, J4 eye LED, J5 speaker, J6 fan, J7 microphone, J8 camera, J9 and J10 the module sockets (pins 1 to 100 and 101 to 200), H1 to H4 the corner holes, H5 the module (outline, keep-out and its M2.5 holes), U2 buck, U3 AMS1117, U4 MAX98357A, U5 BMI088 module, Q1 fan FET. The two sockets are separate parts so JLC places both; the layout must keep them at their exact offsets from H5 (`hardware/kicad/README.md`).

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

## 9.1 Layout, first pass (13/10/2026)

`hardware/head/head.kicad_pcb`, made by `scripts/kicad/gen_head_pcb.py` (placement, from the netlist) and `scripts/kicad/route_head_pcb.py` (hand-drawn copper, Freerouting, pours, stitching), both one-shot bootstraps like the bridge's.

- Placement: no courtyard overlaps. The module (H5) and its sockets (J9, J10) are locked at their fixed offsets. The free block is too small for every header plus a clear camera path, so the microphone, ToF and I2C headers stand in the 5.7 mm strip left of the module, beside the J9 pins they use; head bundle, fan, speaker and eye LED run down the right edge. Buck top right by the head bundle, AMS1117 and amplifier below it, BMI088 module above the camera connector. J8 is turned so its contact pads, the cable entry side on XUNPU's drawing, face the bottom edge.
- Hand-drawn and locked: the three camera pairs (0.2 mm tracks at the socket's 0.4 mm pitch, ground pour either side, bottom plane kept free of tracks beneath them by rule areas) from J10 down the module's edge, under the BMI088 and into J8 from above; and the 5 V from the buck across under the module on the bottom layer, 2.5 mm wide, to vias beside J9's 5 V pins. A first attempt down the left of J9 on the top layer walled in the socket's outer row and was dropped.
- Connector names are on the back silkscreen, which faces up in the head.
- Not finished: Freerouting leaves 11 connections open (SAI2_LRCK, SAI2_SDI, SAI2_SCLK, CAM_SCL and EYE_DATA out of J9; the amplifier's GAIN, SD_MODE and BCLK pins; three short 3.3 V and 5 V joins) and routes most signals on the bottom layer, which cuts up the ground plane. Next: move the amplifier's two resistors beside its pins 2 and 4, route J9's escapes by hand or with the bottom layer weighted against, then review in the GUI.

## 10. Open items

- The CM4 socket positions and mounting holes from Raspberry Pi's CM4 IO board design, confirmed with a printout against a module before ordering.
- The camera cable: a standard opposite-face Pi cable, 50 to 100 mm, pin 1 checked before the first power-up.
- The BMI088's mounted orientation in the head: the board's position and rotation become the MJCF `head_imu` site (`hardware.md` 6.2). The head body's frame in the MJCF is not x-forward: the `head_camera` camera looks along the body's +z, so map the board's axes through that frame, not through the robot's forward, left and up.
- Height under the board for the camera, and over it for the module heatsink.
