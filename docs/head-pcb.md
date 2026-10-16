# Head carrier PCB

Design note for a head PCB that carries the Radxa CM4 directly, in place of the Waveshare CM4-NANO-A and the loose head breakouts: the 6S to 5 V head buck, the 3.3 V regulator, the MAX98357A speaker amplifier, the fan MOSFET, the BMI088 IMU, the camera connector, and JST PH connectors for everything that stays off the board. It replaces the NANO-A, the head buck module and the point-to-point wiring of `hardware.md` sections 4.3, 4.6, 7, 9, 10 and 11.3 in the head. The NANO-A stays on the bench as the board the module is flashed on (`compute-setup.md`).

Date: 15/10/2026. Status: schematic done (section 7.1), ERC clean; layout with the on-board BMI088, the fallbacks, the 1.1 MHz buck and the SPI breakout (sections 6.2, 9.3), DRC 0 violations with two harmless unconnected module ground pins (section 6.2), JLC package regenerated. Not yet reviewed in the KiCad GUI or ordered.

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
- Deferred: a maskrom button and the UART0 debug console. Since rev B (16/10/2026) the reset and maskrom lines and the USB download port are on bottom test pads, so the module can be flashed in place; the NANO-A still works for a loose module.

## 2. Block diagram

```text
 J1 head bundle                                          Radxa CM4 (two DF40 sockets)
  1 +BATT --+-- TVS --+-- bulk C --+-- U1 buck 5 V --+==> 5V ==> pins 77 79 81 83 85 87
  2 GND     |         |            |                  |
  3 bridge TX --> R13 330R ------------------------------)---> UART7_RX  GPIO24  pin 45
  4 bridge RX <-- R14 330R --------------------------------<-- UART7_TX  GPIO23  pin 47
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

The bundle runs 15 to 20 cm up the neck beside servo power (30 cm first assumed). At the input: an SMBJ30A TVS (C19077584, the bridge's part), a 47 µF 50 V aluminium electrolytic (C17, C970679, 6.3 x 7.7 mm, Extended) and two 10 µF 50 V 1210 capacitors (C596319, the bridge's part) at the buck. The electrolytic was added 16/10/2026 on review: with ceramics alone (about 11 µF effective at 25 V) the cable's inductance rings at plug-in into a live pack to well over the pack voltage, and the TVS took every such event; the electrolytic's ESR damps it (TI's datasheet asks for bulk when the supply is more than a few inches away). The TVS clamps at up to about 48 V at its peak current, so the buck needs a rating of 60 V or more, not a 36 V part. No local fuse: F6 on the bridge protects the bundle and this board. The PH header is keyed, so no reverse-polarity part.

### 3.3 5 V buck

TI LMR51635YDDCR (C45262770, Extended, about 3,200 in stock): 4.3 to 60 V in, 3.5 A continuous, synchronous (no external diode), SOT-23-6, 1.1 MHz with light-load PFM. The same TI family as the bridge's LMR51606, one size up. The 400 kHz X version was first chosen for efficiency, but its stock fell to 76 by 13/10/2026; the Y version is the same chip switching faster, a few percent less efficient. Feedback 82k over 15k (both Basic) sets 5.17 V against the 0.8 V reference, a little over 5 V for the drop to the module's pins at 3 A; TI's own pair is 53.6k over 10.2k. The divider's top is on the buck's side of JP1 (section 6.1), so a cut link leaves the buck regulating its own capacitors instead of running open loop. Output three 47 µF 10 V X5R 1206 (C96123, Basic), about 66 µF after bias derating, against TI's 33 µF at 1.1 MHz. Input two 10 µF 50 V 1210 and 100 nF, plus the bulk electrolytic of 3.2 (the 2.2 µF X5R first fitted, worth about 0.7 µF at 25 V, was dropped for it); 100 nF bootstrap. EN tied to VIN. Note the pinout differs from the bridge's LMR51606: 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 CB. Inductor 4.7 µH (TI lists 3.3 µH for 5 V at 1.1 MHz; 4.7 µH gives about 0.8 A ripple at 25 V in): Sunlord SWPA6045S4R7MT (C78804, 5.5 A saturation, 6 x 6 x 4.5 mm), the bridge's inductor series.

Headroom: the high-side current limit is 4.2 A minimum. At 3.5 A out from 24 V the inductor peaks at about 4.0 A, so 3.5 A is the part's edge. Our continuous load is about 3 A and the peaks in 3.1 do not coincide. If the measured draw says otherwise, the step up is the TPS54560B (C1850354, 60 V, 5 A, HSOP-8, non-synchronous, needs a Schottky).

### 3.4 3.3 V

AMS1117-3.3 (C6186, Basic) from 5 V, as on the NANO-A. At 0.4 A it dissipates about 0.7 W, spread into a copper pour on the tab. The module's own 3.3 V output (pins 84 and 86) is used only for GPIO_VREF.

## 4. Module connections

Pi CM4 connector numbering: pins 1 to 100 on the first connector, 101 to 200 on the second. The Radxa CM4 keeps the Pi assignment on these two (`hardware.md` 4.1); its third connector is not fitted on this board. Pin numbers below are from the NANO-A schematic; every row is checked against Radxa's own CM4 schematic v1.20 (sheet 21 for pins 1 to 100, sheet 22 for 101 to 200), not the Raspberry Pi pinout (16/10/2026).

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
| 92 | RUN_PG | RESET_L | TP1 (RST), bottom pad |
| 93 | nRPIBOOT | MASKROM (through R147, 0 Ω) | TP2 (BOOT), bottom pad |
| 103, 105 | USB2_N, USB2_P | USB2_OTG0_DM, _DP, the download port | TP4 (D−), TP5 (D+), bottom pads, with TP6 (G) |
| 46, 54, 28, 31, 30, 34 | GPIO22, 4, 13, 12, 6, 5 | SPI2 M1 MISO, CSN0, MOSI, CLK; GPIO1_C1; GPIO3_A2 | J13 SPI breakout (section 6.2) |
| every GND pin | GND | | ground plane. On the Radxa module every GND pin of both connectors, and the connectors' mounting tabs, join one ground bus (sheets 21, 22), so a socket ground pin left boxed in by its neighbours (J9 pad 23) is still grounded through the module |

Left unconnected, as on the NANO-A: GLOBAL_EN (99, PWRON_L on the Radxa module), nEXTRST (100), USB_OTG_ID (101; the module's ID line floats to device mode, as maskrom wants), EEPROM_nWP (20), WiFi_nDisable (89) and BT_nDisable (91), whose 0 Ω links the NANO-A leaves unfitted. UART0 (pins 55, 51) stays unconnected until the debug console is wanted.

Not used: the I2S MCLK (GPIO16), the spare GPIOs, and the second connector's PCIe, HDMI, DSI and CAM1 pins.

Flashing in place: the RK3576's maskrom protocol runs only over USB2 OTG0. Solder or clip a USB cable's D+, D− and ground to TP5, TP4 and TP6 (leave its 5 V off: the head runs from the pack, and the module holds its own VBUS detect high through R134, Radxa sheet 8). With the head powered, hold TP2 (BOOT) to ground, short TP1 (RST) to ground and release it, then release BOOT; `rkdeveloptool ld` should list the board in Maskrom, and `docs/compute-setup.md` section 5 takes it from there. The stubs are short and unmatched; if high speed will not hold, the port falls back to full speed, slower but enough for flashing.

## 5. Connectors

JST PH 2.0, vertical through hole, hand soldered. Pin 1 is the square pad.

| Ref | Pins | Use | Pin order |
|---|---|---|---|
| J1 | 4 | head bundle from the bridge | 1 +BATT, 2 GND, 3 bridge TX to UART7 RX, 4 bridge RX from UART7 TX: the same order as the bridge's J6, so a straight 1:1 cable. Each UART line passes a 330R (R13 RX, R14 TX) at the module end: it limits current into an unpowered CM4 when the bridge drives the line, and adds margin against ESD on the cable |
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

## 6.2 SPI breakout (15/10/2026)

For a future SPI device, such as a higher-resolution ToF sensor; untested. Of the RK3576's SPI controllers, Radxa's overlays route two to the Pi-carrier pins: SPI1 M0, which the eye LED already uses (its MOSI carries the LED's data, so a second device on it would garble the LED), and SPI2 M1, which is free. Its pins, from the RK3576 pin table and Radxa's device tree for this module on a Pi carrier (`rk3576-radxa-cm4-rpi-cm4-io.dts`, whose pin 11 and 33 labels now agree with what we measured):

| Signal | RK3576 | Pi GPIO | Header pin | CM4 pin |
|---|---|---|---|---|
| CLK | GPIO1_D5 | 12 | 32 | 31 |
| MOSI | GPIO1_C4 | 13 | 33 | 28 |
| MISO | GPIO1_C5 | 22 | 15 | 46 |
| CS0 | GPIO1_C3 | 4 | 7 | 54 |
| INT (spare GPIO) | GPIO1_C1 | 6 | 31 | 30 |
| spare GPIO (reset or enable) | GPIO3_A2 | 5 | 29 | 34 |

CS1 (GPIO1_C2) is the fan's PWM pin, so there is one chip select. The two spare GPIOs are for the device's interrupt or data-ready line and a reset or LPn line, which ToF parts such as the VL53L8CX want. Radxa's overlay `rk3576-spi2-m1-cs0-spidev` enables the controller with a spidev on CS0; its exclusive list names GPIO1_C3, C4, C5 and D5, none of which this board uses otherwise.

J13: nine 1.5 x 2.5 mm wire pads at 2.54 mm on the bottom, under the module along the top edge, away from the camera pairs; wires are soldered from the back, which faces up in the head. Order from the back, left to right: GND, 5V, 3V3, MOSI, MISO, CLK, CS, INT, IO, named on the bottom silkscreen. 3.3 V comes from the AMS1117 (about 0.4 A already in the budget, section 3.1); a device drawing more than a couple of hundred milliamps should take 5 V and its own regulator.

Routing (15/10/2026). The SPI pins sit mid-row on J9's inner side, interleaved with the I2C, eye-LED, fan, camera-control and SAI2 escapes (eleven pins at 0.4 mm), so adding them to the routed board one net at a time left three nets with no room. The board was routed again from placement with J13 in it (`gen_head_pcb.py`, `route_head_pcb.py`, `finish_head_pcb.py`, then `route_one_net.py` for the stragglers, then the tidy). Along the way the tools learnt: the maze router keeps drilled holes and footprint keep-outs (the BMI088's) clear and sees footprint copper such as the solder jumpers' bridges; it uses each net pair's own clearance; the tidy re-stitches any floating piece of pour; `sync_head_pcb.py` brings a routed board up to date with the schematic without re-placing it.

DRC: 0 violations, schematic parity clean, two unconnected pads: J9 pins 32 and 33, two of the module's ground pins. Pin 32 sits between the SPI INT and IO escapes and pin 33 between CLK and CAM_SCL, both fixed by the SoC, and the channel between the socket's rows carries CAM_SDA and a CAM_SCL via, so neither has a path to the pour. The module ties all its ground pins together internally and the carrier connects the other forty-odd, so this is electrically harmless; KiCad's command-line DRC does not honour exclusions for unconnected items, so the report keeps showing them. Moving INT and IO to other spare GPIOs would free pin 32.

## 7. Parts

| Part | Use | LCSC | Class | Shared with the bridge |
|---|---|---|---|---|
| DF40C-100DS-0.4V(51), 1.5 mm stack | module sockets, two | C597931 | Extended | no |
| TI LMR51635YDDCR, 60 V 3.5 A sync buck, 1.1 MHz | U2 | C45262770 | Extended | no (same family) |
| Sunlord SWPA6045S4R7MT, 4.7 µH 5.5 A | L1 | C78804 | Extended | no (same series) |
| Bosch BMI088, LGA-16 3 x 4.5 | U5 | C194919 | Extended | no |
| XUNPU FPC-1.0HF-15PWAR-H20, 15-pin 1.0 mm, dual contact, flip lock | J8 | C19269369 | Extended | no |
| MAX98357AETE+T, TQFN-16 3 x 3 | U4 | C910544 | Extended | no |
| AMS1117-3.3 | U3 | C6186 | Basic | |
| AO3400A | Q1 | C20917 | Basic | |
| B5819W | fan flyback | C8598 | Basic | yes |
| SMBJ30A | input TVS | C19077584 | Preferred | yes |
| 10 µF 50 V X7R 1210 | buck input | C596319 | Extended | yes |
| KT-0603G | power LED | C12624 | Extended | yes |
| RVT1H470M0607, 47 µF 50 V aluminium, 6.3 x 7.7 mm | C17, input bulk | C970679 | Extended | no |

Basic parts besides: 0603 resistors (100R, 330R, 1k, 4.7k, 15k, 82k, 100k, 1M), 100 nF 50 V 0603, 10 µF and 22 µF 25 V 0805, 47 µF 10 V 1206. Every LCSC number is in the schematic.

Not in the BOM: the PH headers, the module itself, the mounting hardware; the solder jumpers are copper only.

Nine Extended types, about AUD 4.30 each in feeder fees (about AUD 39 an order): seven only this board uses (the socket, buck, inductor, FPC, amplifier, BMI088 and the bulk electrolytic; no 50 V SMD electrolytic is Basic or Preferred) and two the bridge also uses (the 10 µF capacitor and the LED), whose fees are paid once only if the two boards are ever ordered together. The MAX98357A's 9-ball WLP (C2682619) is cheaper but a 1.3 mm BGA, not worth it on two layers.

Camera cable: the standard Pi 15-pin camera cable has its contacts on opposite faces at the two ends. A dual-contact connector takes either face, so J8 works whichever way the cable folds under the board; check the pin 1 end with a cable before soldering a camera. A same-face cable reverses the pins and can damage the camera.

Socket footprint: KiCad's library has no DF40 footprint. Raspberry Pi's CM4 IO board KiCad files (`CM4IO-KiCAD.zip`, datasheets.raspberrypi.com) carry one footprint, `Raspberry-Pi-4-Compute-Module`, with both sockets and the four holes in their exact places. Use it rather than drawing the sockets.

## 7.1 Schematic

`hardware/head/head.kicad_sch` (KiCad 10), PDF beside it. Bootstrapped by `scripts/kicad/gen_head_sch.py` in the same way as the bridge's: every pin connects through a net label, power symbol or no-connect flag, and from now on the schematic file is the source (the script refuses to overwrite it without `--force`). Library: `hardware/kicad/DukkiHead.*`, made by `scripts/kicad/gen_head_kicad.py` (`hardware/kicad/README.md`). ERC: no violations. The netlist was checked net by net against sections 4 to 6.

References: J1 head bundle, J2 ToF, J3 I2C breakout, J4 eye LED, J5 speaker, J6 fan, J7 microphone, J8 camera, J9 and J10 the module sockets (pins 1 to 100 and 101 to 200), H1 to H4 the corner holes, H5 the module (outline, keep-out and its M2.5 holes), U2 buck, U3 AMS1117, U4 MAX98357A, U5 BMI088, Q1 fan FET, JP1 the 5 V link, JP3 the amplifier's link, JP4 and JP5 the IMU's, J11 and J12 the fallback wire pads, J13 the SPI breakout pads. The two sockets are separate parts so JLC places both; the layout must keep them at their exact offsets from H5 (`hardware/kicad/README.md`).

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

Tidy (14/10/2026, `scripts/kicad/tidy_head_pcb.py`, run after the routing): Freerouting left small jogs and kinks, and necked tracks down at tight spots; the maze router left a staircase; and the 610 stitching vias, placed after the main autoroute but present for the finishing passes and the maze router, hemmed those in. The tidy removes the stitching, then, wherever two unlocked segments of one net meet at a bare joint, replaces them with one straight segment if it clears every other net's copper by the larger of the two net classes' clearances and the keep-outs (306 joints over four sweeps), widens narrow segments to their net class where they fit, and stitches again more sparingly: a 3.5 mm grid and rows beside the camera pairs, only where both pours are filled (163 vias). DRC with schematic parity stays at 0 violations, 0 unconnected. Tracks are now straight between bends, at any angle. A second look (14/10/2026) found copper crossing ground pads' thermal-relief gaps: Freerouting's GND tracks, routed before any pour existed. All 127 track-on-pad overlaps were ground on ground, so nothing was wrongly connected, but the 215 unlocked GND segments only duplicated the pours; the tidy's `unground` stage removes them. DRC confirms every ground pad still reaches the pour; C9's ground pad needed a solid connection to do so. The six GND tracks left on pads are the hand-drawn, locked ones (the BMI088's ground pins to their vias). A third look found five stitching vias inside ground pads (the amplifier's exposed pad, J8's ground pins 4 and 10, D1 and C8): the stitching kept clear of other nets' pads but not ground's, and a via in an SMD pad wicks solder from the joint. The stitching now keeps clear of every pad. Freerouting had also handed the locked hand-drawn tracks back as second copies (97 duplicates), which the tidy's `dedupe` stage removes. The feed from the pack into the buck runs from U2's VIN pin out through C1's pad and back to C4: not a stray, but the router's way past U2's FB pin, and the order a buck wants (the input capacitor straight at VIN). Segments still below their class width are ground stubs into the pour (0.2 to 0.45 mm against the class's 0.6 mm, which only matters for ground run as track), the hand-drawn BMI088 straps (0.2 mm at its 0.5 mm pitch), the 5 V into the module's pads and the switch-node branch to C5 (0.5 mm), and two 0.25 mm AMP_5V segments with no room to widen; every signal net is at its class width.

Cost (14/10/2026, JLC live prices, 5 boards): parts US$76.30, fees US$34.12 (setup 8, stencil 1.5, 8 Extended types 24, joints about 0.6), about US$110 or AUD 159 for assembly, plus bare boards, shipping and GST. `jlc_cost.py` counts the two socket lines as two Extended types (they share C597931, so JLC charges once) and two joints a part (the sockets have 100 each), so its own total reads US$3 high. The BMI088 is the dearest part, about US$6.56 each.

## 9.4 Rev B: pre-order review (16/10/2026)

- Buck input loop: U2 keeps VIN and GND facing the top edge but moved down 1 mm, so C4 (100 nF) sits straight across those two pins, with U2's GND pin and C4's ground tied to two vias beside it by locked copper; the loop closes in a few millimetres. The boot cap C5 now sits beside the SW and CB pins, the feedback divider under the FB pin, and C1 and C2 side by side right of U2. U2, C1, C2, C4, C17 and D1 connect solid to the pours.
- Input chain in current order down the right: J1, the TVS D1 (now across, its cathode beside J1 pin 1), C17 (47 µF 50 V electrolytic, section 3.2), then the ceramics. C3 (2.2 µF) dropped. The fan parts (Q1, D2, R5, R6) moved down to make room.
- Link UART: 330R series resistors R13 (RX) and R14 (TX) just outside the module edge where UART7 leaves J9; J1's pins 3 and 4 are now nets LINK_RX and LINK_TX.
- Test pads on the bottom under the module beside J9: TP1 RST (pin 92, RESET_L on the Radxa module), TP2 BOOT (pin 93, MASKROM), TP3 G. Short TP1 to TP3 to reset; hold TP2 to TP3 through a reset for maskrom.
- USB download port on three more bottom pads beside J10 (pins 103, 105), labelled USB, D+, D−, G (TP5, TP4, TP6): maskrom needs USB, so without these the BOOT pad was of no use. Added to the routed board with `sync_head_pcb.py` and two hand-drawn stubs (a via each), not a re-route; the generators carry them for any rebuild. Section 4 has the flashing steps.
- J13 moved 2.5 mm right: every pad is now over 6 mm from both module M2.5 holes (pad 9's edge was about 2.9 mm from one, inside a screw head).
- J8 marked "1 GND" and "15 3V3" at its ends (section 10).
- Cut-line keep-outs: a top-layer strip across the gap of each cuttable link (JP1 ext 5 V, JP3 amp, JP4 and JP5 ext IMU), 1.5 mm past the link either side, so a slipping knife meets nothing else. Before it, the fan's +5 V ran along under JP3's gap, a +5 V branch and a ground via sat beside JP1's, and the I2C8 lines entered JP4 and JP5 across theirs; those were ripped up and rejoined by the maze router around the strips. The strips also keep the ground pour out: the link footprint's mask opening spans the whole 0.6 mm gap but its copper bridge only the middle 1.2 mm, and the pour had squeezed a bare 0.2 mm ground sliver into the gap either side of the bridge, beside the 5 V pads. Bottom-layer tracks under the links stay: a knife scoring the top copper does not reach them. `route_head_pcb.py` adds the strips before autorouting.
- Not changed: the eye LED's 3.3 V data into a 5 V WS2812 (out of spec, works on the bench; user's choice). The INMP441 breakout carries its own 100k pull-down on SD.

Routing: route_head_pcb.py and finish_head_pcb.py as before, then tidy (unstitch, unground, dedupe), the maze router for five stranded nets, a second Freerouting pass (finish_head_pcb.py) for SPI2_MISO, which the maze router could not thread, then straighten, widen and stitch. Fixes to the tools on the way: the tidy's straighten and the maze router now treat locked copper of other nets as obstacles (the router had run SPI2_MISO through the 5 V pad over the module's pins, splitting it); unground keeps ground tracks into the module sockets' pads (J9 pad 60 lost its only connection); undangle does one pass per process (it segfaulted) and leaves ground alone; the stitching's search for a via in a floating ground piece steps 0.25 mm, not 1 mm; route_one_net takes `SRC=REF:PAD` and `DST=REF:PAD`, used to join R6's ground pad (its pour piece had no way to the plane). DRC with schematic parity: 0 violations; one unconnected pad, J9 pad 23 (GND, boxed in by its neighbours, as 32 and 33 were; the module joins its ground pins internally; 32 and 33 now connect). Two vias on FAN_N and SPK_P warn as connected on one layer only: each joins two tracks that end inside it, so they stay. 26 BOM lines, 41 placements.

## 10. Open items

- The CM4 socket positions and mounting holes from Raspberry Pi's CM4 IO board design, confirmed with a printout against a module before ordering.
- The camera cable: a standard opposite-face Pi cable, 50 to 100 mm, pin 1 checked before the first power-up. J8's ends are marked 1 GND and 15 3V3 on the silkscreen: with the cable in, check with a meter that J8's 3V3 end reaches the camera's 3V3 before powering.
- The BMI088's mounted orientation in the head: the board's position and rotation become the MJCF `head_imu` site (`hardware.md` 6.2). The head body's frame in the MJCF is not x-forward: the `head_camera` camera looks along the body's +z, so map the board's axes through that frame, not through the robot's forward, left and up.
- Height under the board for the camera, and over it for the module heatsink.
