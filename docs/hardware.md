# Microduck Unitree: hardware and hookup

The hardware for the reworked Microduck: Unitree J288 servos in place of the Dynamixel XL330s, a Radxa CM4 compute module in the head, an STM32G474 bridge that owns the servo bus and the trunk IMU, a BMI088 head IMU beside the ToF sensor, a 6S LiPo, and off-the-shelf breakouts in place of the Pollen HAT. For each subsystem this document gives the design we are building and, as reference, how the shipped Pollen robot does it and where that is known from.

Date: 16/09/2026. Supersedes the 15/09/2026 revision (section 17), the 09/09/2026 revision (section 16) and the 07/09/2026 revision, which kept the Radxa Zero 3W, an RP2350 bridge and a carrier board derived from the HAT (section 15).

## 1. Sources and confidence

Pollen does not publish a wiring guide for the shipped robot. The reference material here is reconstructed from three kinds of source, and each section says which one it leans on.

- Firmware and provisioning code in the `microduck` repository (sibling folder `../microduck`). Device-tree overlays, udev rules, driver code and comments give the bus, address and pin for almost every part.
- The open HAT design, `pollen-robotics/elec_RPI_Robot_HAT` on GitHub (KiCad 9, rev C1, BOM and production schematic PDF). Reference only in this revision: the HAT is not used.
- Vendor documentation: the Radxa CM4 product brief and schematic v1.20, the Radxa CM4 device trees in Radxa's 6.1 kernel tree, the Waveshare CM4-NANO-A wiki, the STM32G474 reference manual and the WeAct core board schematic, the ST LSM6DSV16X datasheet and SparkFun breakout guide, the Bosch BMI088 datasheet and the `bmi088-rs` driver source, the MAX98357A and INMP441 datasheets, Unitree's J288 pages, and the J288 manual and notes in `docs/datasheets/`.

Two things about the shipped robot are not public: the `imu_to_dxl` board design, and the RAM and eMMC fitted to Pollen's development boards. Neither affects the rework.

Pin functions quoted for the Radxa CM4 on Pi-standard connector positions come from the pinmux table in Radxa's schematic, read from a text extraction. The mapping for Pi GPIO12 and GPIO13 was hard to read from that layout; neither is used here, but confirm against the PDF before relying on them.

## 2. Design decisions

| | Shipped Pollen alpha | This rework |
|---|---|---|
| Servos | 15 x Dynamixel XL330-M288-T, TTL bus at 1 Mbps | 15 x Unitree J288, single-wire bus at 6 Mbps, three segments |
| Servo bus master | RK3566 UART2 through the HAT's TTL buffer | STM32G474 bridge, USART in single-wire half-duplex mode |
| Compute | Radxa Zero 3W (RK3566) in the trunk | Radxa CM4 (RK3576, 2 GB) on a Waveshare CM4-NANO-A carrier, in the head |
| Compute to bridge link | Dynamixel Protocol 2.0 on `/dev/ttyS2` | Our own framed protocol, one round trip per tick, full-duplex UART down the neck, 2 Mbps to start |
| Trunk IMU | LSM6DSV16X on the `imu_to_dxl` board, emulating a Dynamixel device at ID 200 | LSM6DSV16X breakout on the bridge's SPI, in the trunk, same 12-byte data block delivered inside the state frame |
| Head IMU | BMI088 on the head module, I2C3 through the HAT's Qwiic port, read by `tofd`, off by default | BMI088 breakout on I2C8 beside the ToF, same addresses, same `tofd` code path, off by default |
| Firmware seam | `DynamixelIo` implements `RobotIo` | New `BridgeIo` implements `RobotIo`; everything above it unchanged |
| Battery | NP-F550 2S Li-ion, 6.6 to 8.2 V | 6S LiPo, 21.0 to 25.2 V |
| 5 V supply | HAT buck (AP63205) in the trunk | One 25 V to 5 V buck in the head, feeding the carrier, the amplifier and, down the neck, the bridge |
| Carrier board | RPI Robot HAT rev C1 | None of our own. Off-the-shelf NANO-A carrier and breakouts. No HAT |
| Audio | HAT codec TLV320AIC3104, PAM8406 amplifier, MEMS mic | MAX98357A I2S amplifier and INMP441 I2S microphone in the head, on the RK3576's SAI2, no codec driver |
| Camera | IMX219 on the Zero 3W's 22-pin CSI | Same sensor on the NANO-A's 15-pin CSI, 2-lane, short cable |
| ToF | VL53L8CX on I2C3 via the HAT's Qwiic port | Same sensor on I2C8 at header pins 3 and 5, in the head, sharing the bus with the BMI088 |
| Radios | AIC8800 on the Zero 3W | AIC8800 on the CM4 module, external antenna in the head |
| Expression outputs | None under software control | Eye: one WS2812-type RGB LED, data from SPI1 MOSI on header pin 19. Head fan: 2-wire 5 V fan on a low-side MOSFET, PWM1 channel 2 on header pin 11. Both driven by `expressd`; see `express-setup.md` |

No Dynamixel emulation anywhere. The bridge speaks J288 frames to the servos and our protocol to the compute module. The reasons are in section 5.4.

## 3. System overview

```text
 HEAD                                                    TRUNK
 +--------------------------------------------+          +------------------------------------------+
 | buck 25 V -> 5 V, 3 A, enable = power sw   |          | 6S LiPo 21.0 to 25.2 V                   |
 |   |-- NANO-A header pins 2/4 (CM4 5 V)      |  +BATT   |   |                                      |
 |   |-- MAX98357A 5 V                         |<---------|  servo power distribution board          |
 |   |-- 5 V down the neck to the bridge       |          |  (XT30/XT60 in, fuse, branches:          |
 |                                            |          |   left leg, right leg, neck+head, buck)  |
 | Radxa CM4 (RK3576) on Waveshare CM4-NANO-A |          |                                          |
 |   pins 16/18  UART7 <----- TX/RX --------->|<-------->|  WeAct STM32G474 bridge                  |
 |   pins 3/5    I2C8  --- VL53L8CX ToF       |  5 V,GND |   USART2 ----> J288 x5 left leg, IDs 0..4|
 |                     --- BMI088 head IMU    |          |                                          |
 |   pins 12/35/40  SAI2 --- MAX98357A + 3 W  |          |   USART3 ----> J288 x5 right leg, 10..14 |
 |   pins 12/35/38  SAI2 --- INMP441 mic      |          |   LPUART1 ---> J288 x5 neck+head, 5..9   |
 |   pins 8/10   UART0 console (debug only)   |  bus C   |               (data up the neck)         |
 |   CSI (CAM0, 2-lane) --- IMX219            |<---------|   SPI ------- LSM6DSV16X at the imu site |
 |   USB-C: flashing only, BOOT switch        |          |   ADC ------- pack voltage divider       |
 |   Wi-Fi 6 / BT 5.4 antenna on the module   |          |   USB-C / SWD: firmware and debug        |
 +--------------------------------------------+          +------------------------------------------+
```

Neck harness, trunk to head: +BATT and GND for the head servo branch, +BATT and GND for the buck, servo bus C data, bridge link TX and RX with a ground, 5 V and GND for the bridge. Section 11.3.

## 4. Compute module and carrier

### 4.1 Radxa CM4

Rockchip RK3576: four Cortex-A72 at 2.2 GHz and four Cortex-A53 at 1.8 GHz, Mali-G52 MC3, a 6 TOPS INT8 NPU driven by the same rknn toolchain as the RK3566, hardware H.264 and H.265 encode and decode through Rockchip MPP, a Rockchip ISP, two MIPI CSI receivers (one 4-lane, one 2-lane) and two PCIe 2.0 x1 lanes. Radxa packages it as a 55 x 40 mm module with three 100-pin board-to-board connectors. The first two connectors are the Raspberry Pi CM4 pair with the Pi pin assignment; the third carries UFS, the second PCIe lane, SATA and DisplayPort, none of which this robot uses. Wi-Fi 6 and Bluetooth 5.4 are on the module (AIC8800 over USB) with an IPEX antenna connector; a no-radio variant exists. The module has its own maskrom button on the component side, at the right edge beside the eMMC.

Why this module rather than the Zero 3W: the same Rockchip software family, so `duck-detect`'s rknn path, `mediad`'s MPP and rkisp pipeline and the Armbian-based provisioning carry over with edits rather than rewrites; roughly twice the CPU and seven times the NPU for perception work; and a CM4-standard footprint that off-the-shelf carriers already serve. Nothing about the control loop needed the change: policy inference is under a millisecond on either SoC and the servo bus sits behind the bridge on both.

Source: Radxa CM4 product brief and schematic v1.20, Radxa docs, sbc-bench results for RK3566 and RK3576 boards.

### 4.2 RAM and storage

2 GB LPDDR4x, the variant available at time of writing. Radxa's SKU list pairs 2 GB with 16 GB of eMMC (RM126-D2E16), and 32 GB with 4 GB. Confirm which eMMC size the ordered part carries; both are sufficient.

- The shipped robot runs the full stack (ONNX Runtime at 50 Hz, GStreamer hardware H.264 and WebRTC, the NPU detector, five daemons) on a 1 GB Zero 3W, so 2 GB has headroom for a second or larger vision model on the NPU. Provision from a minimal CLI image, not a desktop one.
- The updater is application-level: versioned release directories with an atomic symlink swap and a health gate, no A/B root partitions. A minimal root, the release directory, models and the voice bank fit in a few gigabytes on 16 GB with room for a rollback release and logs.
- Expect the vendor kernel to reserve some tens of megabytes of CMA for the NPU and VPU. Keep journald capped as the deploy configuration already does.

Source: `microduck/docs/design/updater-design.md`, Radxa CM4 product brief.

### 4.3 Carrier: Waveshare CM4-NANO-A

55 x 40 mm, the same footprint as the module. It provides the Pi 40-pin header (supplied loose, to be soldered), one 15-pin MIPI CSI connector wired as the Pi's CAM0, one USB 2.0 Type-A, a USB-C for 5 V and flashing, a microSD socket (unused with an eMMC module), power and activity LEDs, and a BOOT switch. The 3.3 V rail comes from an AMS1117 linear regulator good for about 1 A.

Why this carrier: every RK3576 function this design needs is on the header, including a second full UART for the bridge and an I2S port with MCLK for audio, and its BOOT switch does the right thing for the Radxa module (section 4.5). Radxa lists the sibling CM4-NANO-B as a tested carrier; the A is a strict subset of it electrically, minus the Ethernet, HDMI, DSI and the 3.5 mm jack whose PWM filter would have loaded the I2S clock pins.

Notes:

- Feed 5 V into header pins 2 and 4 from the head buck, not through the USB-C. The module can draw more than the USB-C's 2.5 A rating under load, and the USB-C stays free for flashing. Verify on the NANO-A schematic that nothing sits between header 5 V and the module's 5 V input.
- Power nothing hungry from the header's 3.3 V pins. The ToF, I2C pull-ups and the microphone are fine; the bridge has its own regulator.
- The header I2C pins almost certainly have no pull-ups on this carrier. Section 9.

Source: Waveshare CM4-NANO-A and CM4-NANO-B wiki, Radxa CM3J carrier compatibility note.

### 4.4 Operating system

Two workable images; pick one and pin it.

- Armbian. There is a supported `radxa-cm4-io` board config (vendor kernel only, Rockchip 6.1, top support tier, rebuilt August and September 2026). It boots Radxa's CM4 IO board device tree, and Armbian's kernel tree builds only that RK3576 Radxa DTB. For the NANO-A, carry Radxa's Pi-carrier device tree `rk3576-radxa-cm4-rpi-cm4-io.dts` in as an Armbian patch, or select it via `fdtfile` if a later Armbian kernel adds it. This keeps `setup-board.sh`'s Armbian shape and the `radxa-aic8800` Wi-Fi extension.
- Radxa Debian 12 for the CM4. Radxa's own image, `rsetup` for overlays, the same 6.1 vendor kernel. Radxa's dedicated Pi-IO image for the CM4 is a single test build from 27/11/2024; the CM4 IO board image is the maintained one.

Either way the vendor kernel is required: the rknpu driver, rkisp, MPP and the SAI audio controller live there. Mainline RK3576 support exists but lacks the NPU, encoder and CSI, and an open-source NPU driver appeared only in mid 2026 via Mesa rather than rknn.

Overlays this design needs beyond the base Pi-carrier tree: `uart7` on M0 pins for the bridge link, `i2c8` on M1 pins for the ToF, `sai2` on M0 pins with a dummy-codec sound card for audio, and IMX219 on CSI1 with its control I2C on I2C6 (M3 pins, the Pi ID_SD and ID_SC positions) for the camera. The debug console stays on UART0 at header pins 8 and 10, so the console-removal step in `setup-board.sh` is no longer needed.

Source: Armbian `config/boards/radxa-cm4-io.conf`, Armbian `linux-rockchip` rk-6.1-rkr5.1 DTB list, Radxa kernel `linux-6.1-stan-rkr5.1`.

### 4.5 Flashing

The RK3576 selects boot mode from an ADC divider on SARADC_VIN0. Radxa brings that node out on the Pi nRPIBOOT connector position through 22 Ω, so grounding nRPIBOOT at power-on forces USB maskrom mode, the same polarity Pi carriers use. The NANO-A's BOOT switch grounds that line and routes the module's USB OTG0, the RK3576 download port, to the carrier's USB-C.

Procedure: BOOT switch on, apply 5 V, connect the USB-C to a PC, confirm with `rkdeveloptool ld`, write the eMMC with `rkdeveloptool` or RKDevTool, switch BOOT off, power cycle. The Pi `rpiboot` tool and `config.txt` steps in Waveshare's wiki do not apply. The module's own maskrom button does the same job if reachable under the heatsink.

The full procedure, with scripts that build the tools, inject first-boot presets and an SSH key, and write the eMMC, is in `compute-setup.md` and `scripts/compute/`. Verified 19/09/2026 on a 32 GB module.

Source: Radxa CM4 schematic sheet 5 (boot mode config), Radxa CM4 maskrom guide.

### 4.6 Header pins in use

NANO-A header pin, Pi GPIO name, RK3576 function on the Radxa CM4, and what sits there.

| Header pin | Pi GPIO | RK3576 signal | Use |
|---|---|---|---|
| 1, 17 | +3.3 V out | carrier AMS1117 | ToF, BMI088, I2C pull-ups, INMP441 |
| 2, 4 | +5 V in | module 5 V | from the head buck; also feeds the eye LED and the head fan |
| 3 | GPIO2 | I2C8_SDA_M1 (GPIO1_C7) | ToF and BMI088 SDA, add pull-up |
| 5 | GPIO3 | I2C8_SCL_M1 (GPIO1_C6) | ToF and BMI088 SCL, add pull-up |
| 7 | GPIO4 | GPIO1_C3 | spare; used for the first on/off fan test, label `PIN_7` verified |
| 8 | GPIO14 | UART0_TX_M0 | debug console TX |
| 10 | GPIO15 | UART0_RX_M0 | debug console RX |
| 11 | GPIO17 | PWM1_CH2_M1 (GPIO1_C2) | head fan MOSFET gate, see `express-setup.md`. Radxa's device tree labels this line `PIN_33` (see the note below the table) |
| 12 | GPIO18 | SAI2_SCLK_M0 (GPIO1_D1) | I2S bit clock to amplifier and mic |
| 16 | GPIO23 | UART7_TX_M0 (GPIO2_B6) | bridge link, compute TX |
| 18 | GPIO24 | UART7_RX_M0 (GPIO2_B7) | bridge link, compute RX |
| 27, 28 | ID_SD, ID_SC | I2C6_M3 | camera control I2C: the NANO-A's CSI connector SCL and SDA land here, with 4.7 kΩ pull-ups on the carrier. Other addresses on this bus are still free |
| 19 | GPIO10 | SPI1_MOSI_M0 | eye LED data in (WS2812-type, clocked out of SPI), see `express-setup.md` |
| 21, 23 | GPIO9, 11 | SPI1_MISO_M0, SPI1_CLK_M0 | claimed by the SPI1 pin group, unused |
| 24, 26 | GPIO8, 7 | SPI1_CSN0_M0, SPI1_CSN1_M0 | spare (left as GPIO) |
| 29, 37 | GPIO5, 26 | CAN1_M3 | spare |
| 33 | GPIO13 | GPIO1_C4 | spare, no PWM function. Radxa's device tree labels this line `PIN_11` |
| 35 | GPIO19 | SAI2_LRCK_M0 (GPIO1_D2) | I2S frame clock to amplifier and mic |
| 36 | GPIO16 | SAI2_MCLK_M0 (GPIO1_D4) | unused, available |
| 38 | GPIO20 | SAI2_SDI_M0 (GPIO1_D3) | mic data in |
| 40 | GPIO21 | SAI2_SDO_M0 (GPIO1_D0) | amplifier data out |

Radxa's `gpio-line-names` for the header are not all right on this module and carrier: GPIO1_C2 is named `PIN_33` and GPIO1_C4 `PIN_11`, but they reach header pins 11 and 33 respectively (measured 03/10/2026, `express-setup.md` section 3). Measure a pin before relying on its label, and never find one by requesting GPIO lines in bulk: requesting a line remuxes the pin, and a scan that way cut the eMMC off.

Pins 12 and 40 also carry UART10 as an alternate, which is why audio and a third UART cannot both live on this header. Audio wins; the bridge has UART7.

Source: Radxa CM4 schematic v1.20, connector J3A and the RK3576 pinmux table; Pi CM4 pinout for the GPIO positions.

### 4.7 Reference: the shipped Radxa Zero 3W

Rockchip RK3566, four Cortex-A55 at 1.6 GHz, 0.8 TOPS NPU, 1 to 8 GB LPDDR4, optional eMMC, 65 x 30 mm, 22-pin CSI, AIC8800D80 Wi-Fi 6 and BT 5.4 over SDIO and UART HCI. Pollen ships the 1 GB, 32 GB variant with the HAT on its header. Armbian with the `linux-image-vendor-rk35xx` kernel, overlays `uart2-m0` (Dynamixel bus on header pins 8 and 10, console removed from it), `i2c3-pihat` (ToF and codec on pins 3 and 5) and `aic3104-i2c3` (codec). The RK3566's I2S3 bit clock is on header pin 11 where the HAT expects it on pin 12, a mismatch that no longer matters.

Source: Radxa docs, `microduck/README.md`, `microduck/scripts/setup-board.sh`, `microduck/deploy/audio/*.dts`.

## 5. Servos and the bridge

### 5.1 The Unitree J288 bus

From the manual in `docs/datasheets/`, Unitree's product page and the J288/S288 section of Unitree's support site (section 14).

| | Dynamixel XL330-M288-T (shipped) | Unitree J288 (rework) |
|---|---|---|
| Physical layer | TTL single-wire half duplex | TTL single-wire half duplex ("1-Wire", TTL multi-point bus) |
| Baud | 1 Mbps, configurable | 6 Mbps, fixed, 8N1 |
| Protocol | Dynamixel Protocol 2.0, register map, sync read/write | Fixed frames: 20-byte command (0xFE 0xEE header), 26-byte reply (0xFC 0xEE), CRC32 |
| Addressing | IDs 0 to 252, broadcast 254 | IDs 0 to 14, broadcast 15 (no reply). Maximum 15 servos per bus |
| Control | goal position, PID gains in registers | hybrid: tau = tau_ff + kp (p_des - p) + kd (w_des - w), rotor-side units, gear ratio 288.35. The PD closes on the rotor encoder |
| Supply | 3.7 to 6 V rated (run at 2S) | 6.4 to 25.2 V, 25.2 V recommended |
| Feedback | position, velocity, current, voltage, temperature | rotor position, speed and torque, 13-bit output encoder, case and winding temperature, voltage in 0.5 V steps, fault flags |
| Connector | JST EH 3-pin | PH 2.0 3-pin: SIGNAL, VCC, GND |
| Size, mass | 20 x 34 x 26 mm, 18 g | 20 x 34 x 26 mm, 39 g |
| Stall torque | 0.52 N·m at 5 V | 1.5 N·m, specified with torque opposite to speed (braking). Measured peak driving torque at the output: about 0.55 to 0.6 N·m (section 5.1.1) |
| Max speed | | 35 rad/s at 25.2 V, 16.5 rad/s at 12 V, no load |
| Torque constant | | 0.554 N·m/A |
| Current | | 0.45 A no load at 25.2 V, 1.92 A maximum line current |
| Gearbox | | steel spur gears, 288.35:1, backdrivable. Unitree measures about 0.04 N·m at the output to overcome static friction |

Behaviour that matters to the bridge firmware and the actuator model, from the support site:

- Working modes, set by the 3-bit status field: 0 locked or stopped, 1 FOC closed loop, 6 clear faults (sent with tor_des = -256 and the other fields zero), 7 reset (about 0.3 s during which commands are ignored). Mode 0 brakes: measured, it shorts the windings (section 5.1.1). Unitree's recipes: torque control is mode 1 with kp = kd = 0, damping is mode 1 with kp = 0, w_des = 0, tau_ff = 0 and kd set.
- The servo replies only when it receives a command addressed to it, so the poll rate is bounded by the round trip.
- Timeout protection, when its bit is set, stops the servo after about 1 s without a command. Once communication returns, the timeout state has to be cleared by sending the bit as 0 before the servo can be re-enabled.
- Faults latch until a clear command or a reset, and some need a reset or power cycle (overcurrent, rotor encoder, stored data). Power-on takes about 1.3 s.
- Gains are rotor-side. Output-side kp and kd are the rotor values times 288.35 squared. Unitree's quick-start example uses an output-side kp of 0.5 N·m/rad with a small kd, close to the effective kp of 0.55 N·m/rad that the RL project's XL330 model uses today.
- The PD loop reads the rotor encoder, on the motor side of the gear play. The 13-bit output encoder is reported but does not feed the loop. Backlash can be measured directly by comparing rotor position divided by 288.35 with the output encoder.
- The published constants do not agree with an ideal motor: no-load speed gives about 0.72 V per rad/s at the output, against a torque constant of 0.554 N·m/A. Treat both as starting points for identification, not as model parameters.

#### 5.1.1 Measured on the bench

One servo on a 4S supply, 27/09/2026. Tests, tables and method are in `j288-testing.md`; the tools are in `scripts/j288/`.

| | Measured |
|---|---|
| Round trip, one servo on Unitree's USB module | 0.21 ms median, 0.30 ms worst |
| Mode 0 | Shorted windings: a viscous brake of 0.148 N·m·s/rad. A limp joint is damped, not free |
| Reported torque | The motor's current-based torque, before gearbox losses. Within 0.005 N·m of the command |
| Reported speed | Filtered in the servo, first order, about 15 ms. The firmware's kd term uses the filtered speed |
| Torque response | Within 1 ms |
| Motor torque limit | Hard clamp at 1.00 N·m |
| Output torque when driving | Peak at the output about 0.55 to 0.6 N·m, from the pendulum identification. The first scale tests read 0.45 to 0.49 N·m, about 20% low |
| BAM model | M6, bundled in the bam fork as `unitree_j288`; 0.028 rad position error on a held-out load (`j288-testing.md` section 15) |
| Breakaway friction, unloaded | 0.03 N·m |
| Armature | 7.8 × 10⁻⁴ kg·m² |
| Winding heating at stall | About 1 °C/s at 0.5 N·m of motor torque, 4.5 °C/s at the clamp |

What follows for the bridge and the robot:

- Unitree's 1.5 N·m is the back-driven figure. The J288's peak driving torque (0.55 to 0.6 N·m) is about 40% above the XL330 as `microduck_rl` trains with it (0.425 N·m) and level with BAM's current XL330 model, on a heavier robot. In motion the difference grows: the J288 keeps full torque to about 20 rad/s, where the XL330 has lost most of its torque by 10 to 15 rad/s. Section 12 and `j288-testing.md` sections 12 and 15.7.
- Build joint velocities on the bridge from position differences, not from the servo's speed field.
- The damping from kd lags the motion by about 15 ms. At moderate stiffness (kp up to about 2 N·m/rad on a leg joint) it keeps most of its effect; for much stiffer joints the bridge can apply damping itself through tau_ff.
- The timeout latch can take up to ten frames with the bit clear to reset. The bridge must keep sending clearing frames until the reply shows it clear.
- Watch the winding temperature, reported in every reply. It rises far faster than the housing temperature.
- Supply voltage does not change stall torque or heating. It sets the speed up to which full torque is available.
- The output encoder jumps by tens of mrad while the output moves and is only trustworthy at rest.

Fifteen servos fit one bus exactly. This design runs three segments of five from the bridge: segment A for the left leg, segment B for the right leg, segment C for the neck and head up the neck harness. Each limb detaches with one data wire and one power pair, and the bridge polls the three segments in parallel. IDs stay globally unique as `motor-setup.md` assigns them, so a mis-plugged limb cannot collide.

### 5.2 Why the compute module cannot be the bus master

Rockchip's UART controllers on the RK3566 and RK3576 are documented to about 4 Mbps. The J288 baud is fixed at 6 Mbps, so no overlay or clock setting puts the servos on a compute-module UART. A bridge microcontroller is required. Unitree's own examples drive the bus from an STM32F413 with its USART in single-wire half-duplex mode, or from a PC through their single-bus-to-USB module.

### 5.3 Bridge MCU: STM32G474CEU6 on the WeAct core board

The STM32G474 is a Cortex-M4F at up to 170 MHz with 512 KB flash and 128 KB RAM. The WeAct core board as delivered is the QFN48 variant (schematic v1.0 in `docs/datasheets/`, STM32G474CEU6). It carries the MCU with an 8 MHz HSE crystal, a 32.768 kHz LSE, a USB-C connector on the MCU's USB, BOOT0, reset and user keys, a blue LED on PC6, a four-pin SWD header, a W25Q64 8 MB QSPI flash, a VCC input from 3.3 to 20 V regulated to 3.3 V by an LDO of about 250 mA, and the port A and B pins plus PC4, PC6, PC10, PC11 and PC13 to PC15 on two 2 x 12 headers at 2.54 mm. An LQFP48 variant with the same outline exists; section 5.8 notes the differences.

Why it fits:

- Native single-wire half duplex. The USART's HDSEL mode drives and receives on one pin and releases the line when idle, which is how Unitree's STM32F413 example drives the J288. No PIO program and no direction GPIO to time.
- Exact baud rates. Run the core at 168 MHz from the 8 MHz crystal (PLL 8 / 1 x 42 / 2). With 16x oversampling the USART divider is an integer for every rate this design uses: 6 Mbps is 168 / 28, 4 Mbps is 168 / 42, 2 Mbps is 168 / 84. Radxa's UART7 on the other end gets its clock from the RK3576, which reaches 4 Mbps with the right input clock.
- Five usable UARTs on the QFN48 package: USART1, USART2, USART3, LPUART1, and UART4 on PC10 and PC11, which the LQFP48 does not bond out. UART5 needs port C and D pins neither 48-pin package has. Three servo segments and the compute link take four; UART4 shares its pins with the trunk IMU's SPI3 and is the fallback for segment C (section 5.8). The console goes over the USB-C as a CDC device, or over SWD.
- Hardware FPU, 12-bit ADCs for the pack voltage divider, several SPI ports for the IMU, and a driver-enable output on the USARTs if a buffered bus is preferred (section 5.6).
- IO is 3.3 V; the five-volt-tolerant pins accept a 5 V bus signal directly if the J288 turns out to drive one.
- Rust support through `embassy-stm32` or `stm32g4xx-hal`, so the bridge firmware can share language and tooling with `microduck`.

Pin allocation and wiring are in section 5.8.

Alternative, retained as a fallback: the RP2350 (Pico 2 or RP2350-Zero). Its fractional UART divider also hits 6 and 4 Mbps exactly, and a PIO state machine implements the single-wire bus with pin-direction flipping. It has only two hardware UARTs, no five-volt-tolerant pins, and the A2 silicon's E9 pull-down erratum. The Teensy 4.x and ESP32 family were excluded for the baud-rate reasons recorded in the previous revision.

Source: STM32G474 datasheet DS12288 and RM0440, `docs/datasheets/WeAct-STM32G474CoreBoard_V10_SchDoc.pdf`.

### 5.4 Protocol and firmware seam

The bridge speaks our own protocol to the compute module. No Dynamixel emulation.

Why:

- `duck-control` already has the right seam. The `RobotIo` trait (`duck-control/src/io.rs`) has `read()` returning joint positions, velocities, currents and IMU data, `write()` of joint targets, `set_torque`, `set_gain`, `slow_sensors()` for pack volts and per-joint temperatures, and `imu_stale` and `imu_ready`. `DynamixelIo` is one implementation and `FakeIo` another. Everything above the trait (observations, policy, safety, limp-fall, IPC, health, the update gate) never sees a bus packet. A `BridgeIo` implementation over a serial port is a few hundred lines of Rust.
- Emulating sixteen Dynamixel register maps on the MCU would force J288 semantics (multi-turn rotor position, hybrid control, torque feedback) through an XL330-shaped hole (single-turn ticks, a P gain, current), with Protocol 2.0 byte stuffing and strict reply ordering on top. Two protocol layers to debug instead of one.
- The J288's hybrid control (tau_ff, p_des, w_des, kp, kd) is available end to end for the RL side to use.

What is given up: `rustypot`'s scan and wizard tools, `robotd init`'s direct-bus path, and running Pollen's stock releases. The first two are small tools to rewrite; the third is gone the moment the servos change.

Frame sketch, one round trip per 50 Hz tick, fixed length, sequence number, CRC-16 or CRC-32. A specification, not this sketch, goes in a separate bridge-protocol document.

| Direction | Contents | Approximate size |
|---|---|---|
| Compute to bridge | seq, mode flags (torque on, per-joint enable), 15 x (p_des, w_des, kp, kd, tau_ff) in output-side SI units | about 300 bytes |
| Bridge to compute | seq, 15 x (position, velocity, torque, temperature, fault bits), IMU block (12 bytes, section 6), pack voltage from the bridge's ADC, bridge status | about 250 bytes |

At 2 Mbps that is about 2.8 ms of link time per tick, and at 4 Mbps about 1.4 ms. The fifteen J288 exchanges cost the bridge roughly 1.2 to 1.5 ms of bus time at 6 Mbps on one segment, or about 0.5 ms with the three segments served in parallel. All well inside the 20 ms tick, and inside a 10 ms tick if the control rate is ever raised.

Firmware changes in `microduck` (a fork):

- `duck-control/src/bridge.rs` (new): `BridgeIo` implementing `RobotIo`, plus the `init` and `relax` operations `robotd init` needs (today those call `DynamixelIo` inherent methods).
- `duck-control/src/model.rs`: bus IDs 0 to 14, battery constants for 6S (section 10). Joint order, names and `DEFAULT_POSITION` stay; the RL contract depends on them.
- `duck-control/src/imu.rs`: unchanged if the bridge ships the same 12-byte block. The mount quaternion becomes a parameter.
- `robotd-params` and `deploy/robotd.toml`: `[bus]` gains the protocol and baud, and points at the new port.
- `set_gain` today writes an XL330 P gain of 200 and the standing and limp-fall modes scale it. For the J288 this becomes kp and kd parameters, and it is a retune, not a unit conversion.
- `currents_ma` in `Sensors` has no J288 equivalent. Report torque, or derive a current estimate, and check what consumes the field before deciding.

### 5.5 Link to the compute module

UART7 on NANO-A header pins 16 (compute TX, GPIO2_B6) and 18 (compute RX, GPIO2_B7), wired to one of the G474's USARTs. Both sides 3.3 V, full duplex, no buffer. An overlay enabling `uart7` on its M0 pins is required; on the vendor kernel the port appears as `/dev/ttyS7` (confirm on the board) and `robotd.toml`'s `[bus] port` points at it. The debug console stays on UART0, `/dev/ttyS0`, at header pins 8 and 10, so nothing needs removing from it.

The link runs about 30 cm down the neck harness beside servo power. Run TX and RX each as a twisted pair with ground, start at 2 Mbps, and treat 4 Mbps as a bench gate with a scope on the far end. If 2 Mbps holds and 4 Mbps does not, 2 Mbps is the design rate; the tick has room for it.

Both directions are level-compatible with no translation: the RK3576 header pins and the G474 pins are 3.3 V.

### 5.6 Bus electrical

- Single-wire, 6 Mbps, a bit is 167 ns. Two ways to drive it from the G474.
  - HDSEL mode direct. ST's reference manual specifies the TX pin as alternate-function open-drain with an external pull-up in this mode, since the pin is released when not transmitting. Size the pull-up for the segment's capacitance: the segment must rise well within a bit time, so expect something in the low kilohm range for a five-servo segment, and verify the edge with a scope. Unitree's example uses this mode.
  - Buffered. Keep the USART full duplex and use its driver-enable output to switch a 74LVC1G125 driver and 74LVC1G126 receiver pair, the parts the HAT used, with about 4 ns propagation. Do not reuse the HAT's PNP auto-direction network, whose RC time constants were chosen for 1 Mbps. This is the safe default until the J288's signal level is measured, since 74LVC inputs are 5 V tolerant.
- Signal level. The J288 manual does not state the SIGNAL logic level; Unitree's STM32F413 example is a 3.3 V part connected through an adapter board. Confirm with a scope before connecting a G474 pin directly, and if direct, use a five-volt-tolerant pin.
- One pull-up per segment on the data line, a series protection resistor and a clamp as on the HAT (150 R, 5.1 V zener).
- Topology: segments A and B each star from the bridge in the trunk to the five servos of one leg. Segment C's data line runs up the neck harness to the five neck and head servos. Power is not run through the servo pigtails (section 10).

### 5.7 Reference: the shipped Dynamixel bus

For context, since the HAT's Dynamixel section is what this design replaces.

- Port `/dev/ttyS2`, RK3566 UART2 M0 on header pins 8 and 10, `uart2-m0` overlay, kernel console removed from it and `serial-getty@ttyS2` masked. 1 Mbps, Dynamixel Protocol 2.0 via `rustypot` with `TIOCEXCL`.
- Sixteen devices: servos at IDs 10 to 14 (right leg), 20 to 24 (left leg), 30 to 34 (neck, head, mouth) and the `imu_to_dxl` board at ID 200. One `sync_read` of registers 124 to 135 across all sixteen and one `sync_write` of goal positions per tick, voltage and temperature (144 to 146) once a second. Startup pins `return_delay_time = 0`, `baud_rate = 3`, `pwm_slope = 255`, `shutdown = 52`.
- The HAT's half-duplex circuit follows the ROBOTIS Dynamixel Shield: direction derived from TX through a PNP (MMBT3906) and 10 k/20 k network, SN74LVC1G125 to the line, SN74LVC1G126 and a 74LVC1G08 back, 150 R series, 5.1 V zener, two JST EH 3-pin connectors with the battery rail through a 0 R link marked 3 A. An RS485 path (SIT3088E, two JST EH 4-pin) is present and unused.

Source: `microduck/duck-control/src/bus.rs`, `model.rs`, `deploy/robotd.toml`, `docs/design/robotd-design.md` §1.1, HAT schematic sheet 4/6.

### 5.8 Bridge hookup

Everything that plugs into the WeAct board, by G474 pin. Pin names are what the board's silkscreen shows, and the header column gives the schematic connector and pin (P1 is the VCC side, P2 the 3V3 side; odd pins are the inner row). The I/O type is from the G474 datasheet pin table: FT pins are 5 V tolerant, TT pins are 3.6 V tolerant. The allocation puts each servo data line on an FT pin so the direct single-wire drive of section 5.6 is possible on all three segments, keeps USB, SWD and the board's own LED and keys free, and leaves the six pins of the on-board flash chip unused.

```text
                         WeAct STM32G474 core board (QFN48, STM32G474CEU6, two 2 x 12 headers)
                         +--------------------------------------------------+
 5 V, GND (neck) ------->| VCC, GND        LDO -> 3.3V pins                 |---> 3.3 V to IMU, pull-ups, buffers
 NANO-A pin 18 UART7_RX <| PA9   USART1_TX                                  |
 NANO-A pin 16 UART7_TX >| PA10  USART1_RX                                  |
                         |                                                  |
 segment A data <------->| PB3   USART2_TX  single-wire  [PB4 RX, PA1 DE]   |  left leg, IDs 0..4
 segment B data <------->| PB9   USART3_TX  single-wire  [PB8 RX, PB14 DE]  |  right leg, IDs 10..14
 segment C data <------->| PA2   LPUART1_TX single-wire  [PA3 RX, PB12 DE]  |  neck and head, IDs 5..9, up the neck
                         |                                                  |
 LSM6DSV16X SCL/SCLK <---| PC10  SPI3_SCK                                   |
 LSM6DSV16X AD0/MISO --->| PC11  SPI3_MISO                                  |
 LSM6DSV16X SDA/MOSI <---| PB5   SPI3_MOSI                                  |
 LSM6DSV16X CS       <---| PA15  GPIO                                       |
 LSM6DSV16X INT1     --->| PA8   GPIO, EXTI                                 |
 +BATT 100k/10k divider >| PA0   ADC1_IN1                                   |
                         |                                                  |
                         | PA13, PA14  SWD header     PA11, PA12  USB-C     |
                         | PC6 blue LED   PC13 user key   PB8 BOOT0 key     |
                         | PB2 VBUS sense (solder bridge SB4)               |
                         | PA6 PA7 PB0 PB1 PB10 PB11  W25Q64 flash, unused  |
                         +--------------------------------------------------+
```

| G474 pin | Header | Type | Function | Connects to | Notes |
|---|---|---|---|---|---|
| PA9 | P1-11 | FT_fda | USART1_TX, AF7 | NANO-A header pin 18 (UART7_RX) | compute link, 2 Mbps to start (section 5.5) |
| PA10 | P1-12 | FT_fda | USART1_RX, AF7 | NANO-A header pin 16 (UART7_TX) | ground runs with the pair |
| PB3 | P1-18 | FT | USART2_TX, AF7 | segment A data, left leg | single-wire drive: this pin only |
| PB4 | P1-19 | FT_c | USART2_RX, AF7 | 74LVC1G126 output | buffered variant only. UCPD dead-battery pull-down at reset, disable it in firmware |
| PA1 | P2-17 | TT_a | USART2_DE, AF7 | 74LVC1G125 and 126 enable pins | buffered variant only |
| PB9 | P1-24 | FT_f | USART3_TX, AF7 | segment B data, right leg | single-wire drive: this pin only |
| PB8 | P1-23 | FT_f | USART3_RX, AF7 | 74LVC1G126 output | buffered variant only. Shared with the BOOT0 key and its 10 k pull-down; BOOT0 is sampled only at reset |
| PB14 | P1-7 | TT_a | USART3_DE, AF7 | 74LVC1G125 and 126 enable pins | buffered variant only |
| PA2 | P2-18 | FT_a | LPUART1_TX, AF12 | segment C data, neck and head | single-wire drive: this pin only |
| PA3 | P2-15 | TT_a | LPUART1_RX, AF12 | 74LVC1G126 output | buffered variant only |
| PB12 | P1-5 | TT_a | LPUART1_DE, AF8 | 74LVC1G125 and 126 enable pins | buffered variant only |
| PC10 | P1-16 | FT | SPI3_SCK, AF6 | IMU SCL/SCLK | |
| PC11 | P1-17 | FT_f | SPI3_MISO, AF6 | IMU AD0/MISO | the chip's SDO |
| PB5 | P1-20 | FT_f | SPI3_MOSI, AF6 | IMU SDA/MOSI | the chip's SDI |
| PA15 | P1-15 | FT_f | GPIO output | IMU CS | JTDI with the internal pull-up at reset, so CS idles high until firmware takes the pin; low selects SPI and the device |
| PA8 | P1-10 | FT_a | GPIO input, EXTI | IMU INT1 | data ready, rising edge |
| PA0 | P2-20 | TT_a | ADC1_IN1 | pack voltage divider | 100 k from +BATT, 10 k to ground, 100 nF at the pin |
| PA11, PA12 | P1-13, P1-14 | | USB DM, DP | board USB-C | DFU bootloader and CDC console |
| PA13, PA14 | SWD header P3 | | SWDIO, SWCLK | board SWD header P3 (3.3V, SWDIO, SWCLK, GND) | |
| PC6 | P1-9 | FT_f | GPIO output | board blue LED through 5.1 k | active high |
| PC13 | P2-24 | FT | GPIO input | board user key to 3.3 V through 330 R | high when pressed, needs a pull-down |
| PB8 | P1-23 | | BOOT0 | board BOOT key, 10 k pull-down | hold at reset for DFU |
| PB2 | P2-7 | TT_a | ADC2_IN12 | VBUS through 100 k/10 k and solder bridge SB4 | USB-present sense, if SB4 is closed |
| PA6, PA7, PB0, PB1, PB10, PB11 | P2-14, 11, 9, 10, 5, 6 | | QUADSPI1 | on-board W25Q64 flash | not used, see below |
| PA4, PA5, PB6, PB7, PB13, PB15, PC4 | | | spare | | I2C1 on PB6 and PB7; SPI2 SCK and MOSI on PB13 and PB15 |

Servo segments.

- Direct single-wire drive: wire the TX pin only, configured as alternate function open-drain, with the segment pull-up to 3.3 V at the bridge, a 150 R series resistor and a 5.1 V zener at the data connector as on the HAT. The RX and DE pins in brackets stay free. The three TX pins are FT, so a 5 V bus idle is safe if the J288 turns out to drive one (section 5.6).
- Buffered drive: TX to the 74LVC1G125 input, its output to the data line, the data line to the 74LVC1G126 input, its output to RX, and the USART's DE pin to both enable pins. The 125 enables on a low and the 126 on a high, so one line switches direction, and DE must be low while transmitting: set the polarity bit (DEP = 1) in CR3. The buffers run from the board's 3.3 V. The same TX pins serve both variants, so a segment can change drive without rewiring.
- The bridge-end connector per segment carries data and ground only. The pigtail's VCC goes to the distribution board (section 10), never to the WeAct board.
- LPUART1 supports single-wire half duplex and driver enable like the USARTs. Clock it from PCLK1 or SYSCLK at 168 MHz; its divider is 256 x 168 / 6 = 7168 at 6 Mbps, an integer. Unitree's example uses a full USART, so bench segment C first. If LPUART1 disappoints, segment C moves to UART4, a full UART that the QFN48 package brings out: TX on PC10 (FT), RX on PC11, DE on PA15 (AF8), divider 28 at 6 Mbps. The IMU then moves to SPI2 on PB13 (SCK), PB14 (MISO) and PB15 (MOSI) with CS on PA4, and segment B gives up its hardware DE pin (PB14) and must use direct drive.
- Ground. The bridge's ground arrives down the neck from the head buck (section 10), while the servo grounds are on the distribution board beside it. Add a short ground strap from the bridge to the distribution board so the data lines are referenced to the servo ground rather than through the neck loop.

On-board flash. The QFN48 board carries a Winbond W25Q64 (8 MB, U3) on QUADSPI1: CS on PB11 with a 100 k pull-up to 3.3 V (R11), CLK on PB10, and IO0 to IO3 on PB1, PB0, PA7 and PA6. All six pins also reach header P2. It held WeAct's USB mass-storage demo as delivered. The allocation leaves all six pins unused so the chip stays deselected: firmware must never drive PB11 low, and with CS high the chip's data pins are high impedance or inputs. Desoldering U3 would free the six pins; nothing in this design needs them. The chip could later hold parameters or logs.

Trunk IMU breakout. The module on hand (`docs/datasheets/lsm6dsv16xtr.png`) has a seven-pin row, a three-pin row, a SOT-23-5 regulator, I2C pull-ups and the X and Y axes on the silkscreen.

| Breakout pin | Connects to | Notes |
|---|---|---|
| VCC | WeAct 3.3V | 3.3 V is safe whether the regulator is a 3.3 V LDO (its output sags to about 3.1 V, inside the chip's 1.71 to 3.6 V range) or a pass-through. Do not feed 5 V until the regulator is identified and its output measured |
| GND | GND | |
| SCL/SCLK | PC10 | SPI clock |
| SDA/MOSI | PB5 | SDI |
| AD0/MISO | PC11 | SDO. The I2C address pin doubles as the SPI output |
| CS | PA15 | low selects SPI; the chip is in I2C mode while CS is high |
| INT1 | PA8 | data ready |
| INT2, SCX, SDX | not connected | second interrupt, and the sensor-hub I2C for an external magnetometer |

Configure 4-wire SPI (SIM = 0) at 10 MHz or less, and set I2C_disable in IF_CFG after the first access so a glitch on CS cannot drop the chip back to I2C. The module's pull-ups sit on SCL and SDA; on SPI they hold SCK and SDI high when idle and cost nothing.

Power.

- 5 V and ground from the neck harness to VCC and GND (the first four pins of P1 are two VCC and two GND). VCC feeds the board's 3.3 V LDO; the QFN48 schematic lists an ME6216A33 with an ME6239A33 (250 mA) as the alternative, so plan on 250 mA. The G474 at 168 MHz, the IMU, three segment pull-ups and the buffers are under 100 mA, so the LDO runs cool at 5 V in.
- USB VBUS reaches VCC through a Schottky diode (D4, B5819W), so the USB-C and the neck 5 V can both be connected on the bench. Solder bridge SB10 shorts that diode; leave it open.
- The pack divider is on the pack whenever the pack is connected, drawing 0.23 mA at 25.2 V and putting about 2.3 V on PA0 through 100 k while the bridge is unpowered, which the pin tolerates. Full scale is 36 V, so a 25.2 V pack reads 2.29 V.

Programming and debug.

- USB-C for the ROM DFU bootloader (hold the BOOT key, tap reset) and a CDC console at run time; `src/bridge/flash.sh` builds the bridge firmware and flashes it with `dfu-util`. SWD header P3 for a probe and RTT logging.
- The board's solder bridges SB3, SB5, SB6 and SB7 join PA9 and PA10 to PB6, PB4 and the USB-C CC pins for USB-PD work. They must stay open, or the compute link on PA9 and PA10 is tied to segment A's RX. Check them on the delivered board along with SB4 (VBUS sense) and SB10.
- PB4 and PB6 carry the G474's UCPD dead-battery pull-downs at reset. Clear them (UCPD1_DBDIS in PWR_CR3) early in firmware; PB4 is USART2's RX in the buffered variant.

The other variant. WeAct also make this board with the LQFP48 part (STM32G474CET6) on the same outline and header grid. On it the four header positions carrying PC4, PC6, PC10 and PC11 here are not connected, the blue LED is on PA8 and there is no flash chip. This section is written for the QFN48 board as delivered; on the LQFP48 board the IMU could not use SPI3 and would go back to SPI1 on PA5, PA6 and PA7.

Source: `docs/datasheets/WeAct-STM32G474CoreBoard_V10_SchDoc.pdf` and `docs/datasheets/WeAct-STM32G474CoreBoard_V10 Board Shape 外形.pdf`; STM32G474 datasheet DS12288 pin definitions and alternate-function tables, cross-checked against the alternate-function tables embassy-stm32 0.6 generates for the STM32G474CE; RM0440 USART and LPUART chapters; `docs/datasheets/lsm6dsv16xtr.png`; LSM6DSV16X datasheet.

## 6. IMUs

Two, as on the shipped robot. The trunk IMU is what the control loop, fall detection and the policy observe. The head IMU is a separate sensor for mapping and vision work, served by `tofd` on request and off by default.

### 6.1 Trunk IMU: LSM6DSV16X on the bridge

One ST LSM6DSV16X, the same part the shipped robot uses, on a breakout in the trunk, read by the G474 over SPI and delivered to the compute module inside every state frame. The SFLP engine on the chip does the fusion; neither the bridge nor the compute runs a filter.

Breakout: the generic LSM6DSV16X module on hand (`docs/datasheets/lsm6dsv16xtr.png`), a small board with a seven-pin row (VCC, GND, AD0/MISO, SDA/MOSI, SCL/SCLK, CS, INT1), a three-pin row (SDX, SCX, INT2) for the sensor hub and a second interrupt, an on-board SOT-23-5 regulator and I2C pull-ups, with the chip's X and Y axes marked on the silkscreen. With CS low the chip runs 4-wire SPI at up to 10 MHz. Supply 3.3 V from the WeAct board; the pin map and the regulator caution are in section 5.8. The SparkFun 6DoF IMU Breakout (Qwiic) and ST's STEVAL-MKI227KA adapter are alternatives with the same signals; on the SparkFun board the address jumper must be fully opened for SPI.

Wiring to the G474: SPI3 on PC10 (SCK), PB5 (MOSI) and PC11 (MISO), CS on PA15 and INT1 on PA8 for data-ready, five signals plus 3.3 V and ground (section 5.8). Mount the breakout where the RL model puts the `imu` site, (-0.021, 0.000, -0.0147) m in the trunk frame, on a rigid part of the trunk, not on the bridge board if the bridge board floats on standoffs.

Keep the data block the shipped firmware already decodes, so `duck-control/src/imu.rs` stays as is:

| Bytes | Contents |
|---|---|
| 0 to 5 | gyro x, y, z as little-endian i16 raw counts at ±500 dps (17.5 mdps/LSB) |
| 6 to 11 | SFLP game-rotation quaternion x, y, z as IEEE half floats; w = sqrt(1 - x² - y² - z²), positive |

Chip configuration, inferred from the host's assumptions: SFLP game rotation vector (no magnetometer), gyro full scale ±500 dps, SFLP output around 100 Hz (the host waits about 25 samples before trusting orientation), gyro bias estimation on. ST's FIFO emits the game rotation vector as exactly three fp16 values with w positive, so the format is a copy, not a conversion. Accelerometer full scale is unobservable from the host and free to choose.

Behaviour the host relies on, to reproduce on the bridge:

- All-zero quaternion bytes mean "SFLP has not started"; the host holds its last value. Send zeros until the first fused sample exists.
- The host flags the IMU as frozen after 25 identical blocks. Update the block from the sensor FIFO on the bridge at the sensor's rate and never block a reply on a sensor read.
- The host applies a fixed mount rotation (`SflpDecoder::DEFAULT_MOUNT`, +90° about Y for Pollen's placement). The bridge sends raw chip frame data; the mount quaternion for our placement is derived on the bench and promoted to a `robotd.toml` parameter. This is the calibration that fails quietly: wrong, it produces a robot that walks badly rather than one that reports a fault.

Bridge implementation, verified on the bench 03/10/2026 (`src/bridge`). ST's `lsm6dsv16x-rs` driver (2.1.0, async, BSD-3) configures the chip over SPI3 at 8 MHz with DMA: accel ±4 g and gyro ±500 dps, both 120 Hz high-performance; SFLP game rotation at 120 Hz with its gyro bias started from zero; gyro, game rotation, gravity and gyro bias batched into the FIFO in stream mode; FIFO threshold of four entries (about 8 ms) on INT1; I2C disabled after the first access. Each INT1 drains the FIFO into the 12-byte block above, copying the gyro and game-rotation bytes as they come. Reading the IMU and replying never wait on each other. On the bench the gyro and game-rotation streams both arrived at 120 Hz with no FIFO overruns or INT1 timeouts, the SFLP gravity vector stayed at 1.00 g through rotation, and the fused roll rate matched the gyro. At rest for three minutes the SFLP gyro bias estimate held at (-337, -96, +687) mdps, within 5 mdps of the measured gyro offset (-340, -98, +691), and the fused attitude drifted 0.08°/min in yaw (about 5°/h) and under 0.07°/min in roll and pitch, with 0.2° or less of noise; gravity read 999.4 ± 0.3 mg.

The driver depends on `bisync`, which its author archived and yanked from crates.io on 24/08/2026. Until ST releases a fix (STMicroelectronics/lsm6dsv16x-rs issue 3), the bridge's `Cargo.toml` pins it to its last release commit on GitHub with a `[patch.crates-io]` entry; remove the patch when a fixed driver is published.

Source: `microduck/duck-control/src/imu.rs`, `bus.rs`, `microduck_rl/.../robot_groundcontact.xml`, `docs/datasheets/lsm6dsv16xtr.png`, LSM6DSV16X datasheet.

### 6.2 Head IMU: BMI088 on I2C8

A Bosch BMI088 module in the head, on the same I2C bus as the ToF sensor, read by `tofd` as the shipped firmware reads it. Kept to stay consistent with Pollen's build: the firmware since 0.11.0 serves it as `head_imu.stream`, the kinematics crate places its samples in the trunk frame from the RL model's `head_imu` site, and the mapping work upstream will assume it exists.

The module ordered (AliExpress item 1005013089348960, sold as "BMI088 6-Axis IMU Module, I²C/SPI, 3.3-5.4V" by Shenzhen Module Studio, in its I2C variant) is a generic 0.1 inch header board, about 20 x 15 mm with two mounting holes at diagonal corners. One edge carries VCC, GND, SCK, SDA, SDO, csA and csG; the other carries INT4, INT3, INT2 and INT1. A SOT-23-5 regulator makes the chip's 3.3 V from VCC. The back of the board labels the same pins for SPI as csG, csA, MO, MI, SCK, GND, VCC, so SDA is MOSI and SDO is MISO in SPI mode.

Three things about this board decide the wiring:

- The interface is set by a 0 Ω link beside the chip on the PS pin, under the "IIC" silkscreen. The board as delivered (03/10/2026) had the link in the SPI position despite the I2C variant being ordered. PS only selects the gyroscope's interface (the accelerometer always starts in I2C and switches to SPI on a rising edge of CSB1), so in that state the accelerometer answers at 0x18 and the gyroscope is absent at both 0x68 and 0x69. Moving the link to the IIC pads fixed it. Check it on any replacement board.
- One SDO pin serves both halves of the chip. For SPI both parts share the MISO line, so SDO1 and SDO2 are almost certainly joined to the one pin (confirm with a meter between the SDO pin and chip pads 10 and 15). In I2C mode that pin sets both address bits at once: SDO to ground gives accelerometer 0x18 and gyroscope 0x68, SDO to 3.3 V gives 0x19 and 0x69. The driver's 0x19 and 0x68 pair cannot be produced. Tie SDO to ground, the datasheet's default connection, and fix the driver (below). Do not leave SDO floating; the datasheet requires unused inputs at a defined level.
- csA and csG (CSB1 and CSB2) are unused in I2C mode. The datasheet allows CSB1 at VDDIO or floating and CSB2 floating. Leave both unconnected. INT1 to INT4 are not used by the driver and the datasheet says not to connect unused interrupt pins.

Driver change. `bmi088-rs` v0.1.2 uses two constants, 0x19 and 0x68, directly in every read and write. The `configurable-addresses` branch of a sibling checkout (`../bmi088-rs`, from tag v0.1.2) stores the addresses in the driver and adds `Bmi088::with_addresses` and `Bmi088::detect`. `detect` reads the chip-ID registers (accelerometer 0x1E, gyroscope 0x0F), trying 0x19 before 0x18 and 0x68 before 0x69, and only reads, so on a HAT the codec at 0x18 is never written to. `Bmi088::new` keeps its old behaviour. The same branch fixes `read_temperature`, which read the signed 11-bit register as unsigned, so anything below 23 °C came out near 280 °C (a bug on Pollen's robots too). `tofd`'s `open_imu` calls `detect` and then `with_addresses`, and logs the pair it found. No config key is needed, so `robotd.toml` is unchanged and a stock duck behaves exactly as before. The microduck workspace `Cargo.toml` (local `dukki` branch) has a `[patch]` entry pointing the pinned git dependency at `../bmi088-rs`; drop it once the change is merged and tagged upstream. Offer it to `pollen-robotics/bmi088-rs` and `microduck`, since any board other than the HAT's own layout hits the same problem. This is the only firmware change the head IMU needs.

The rest is unchanged from the shipped design:

- Interface. I2C at 400 kHz is fine for both the BMI088 and the ToF. The ToF's firmware upload at every start shares the bus, but the kernel serialises transactions and the IMU thread backs off and retries around it.
- Configuration. The driver's defaults are ±3 g at 100 Hz bandwidth for the accelerometer and ±500 dps at 100 Hz for the gyro, fused by a Madgwick filter with beta 0.1. Temperature is read every hundredth sample. Nothing to set on the board.
- Supply. Feed VCC with 3.3 V from header pin 1 or 17, as for the ToF, so nothing at 5 V comes near the RK3576's header. The onboard regulator will sit a little below 3.3 V with that input, which is inside the chip's 2.4 to 3.6 V VDD range, and its output is what sets the IO level. Check with a meter that the regulator output, not VCC, feeds the chip's VDDIO and any onboard pull-ups before connecting SDA and SCL.
- Pull-ups. Section 9. The board may carry its own SDA and SCL pull-ups (there is an unmarked resistor network beside the IIC pad); measure SDA to VCC unpowered. Add up what is on the bus and aim for an effective 2.2 k to 4.7 k at 400 kHz.

Wiring to the NANO-A header: VCC to pin 1 or 17, GND to ground, SCK to pin 5, SDA to pin 3, SDO to ground. Five wires. csA, csG and the four INT pins stay open.

Placement. The RL model puts the `head_imu` site on the head body at (0.0152, 0.0001, -0.0511) m with an identity quaternion, so its axes are the head body's axes. The camera site on the same body is rotated, which is why the firmware describes the IMU as tilted relative to the camera. Mount the breakout so its chip axes match the head body frame, or, if the head layout forces another orientation, change the site quaternion in the MJCF rather than the firmware: `tofd` publishes raw chip axes and every consumer rotates by the site pose the daemon serves in `robot.state.frames.head_imu`. The head body's mass and inertia already need updating for the compute module (section 11.1); add the breakout's few grams to that.

Firmware. No other code changes. Set `[head_imu] enabled = true` in `robotd.toml` through `robotctl configure`, which offers the `tofd` restart. `tofd --imu` reads the chip for one session without touching the file, `--no-imu` forces it off, and `--imu-hz` trades rate for CPU. Reading at 100 Hz costs 3.5 to 4.5 percent of one Cortex-A55 core on the RK3566, almost all of it the two I2C transactions a sample takes; expect less on the RK3576 but not zero. Off by default upstream because nothing on the robot subscribes yet, and the same default is right here. `sudo i2cdetect -y -r 8` should show 0x18, 0x29 and 0x68 once it is wired, and `journalctl -u tofd` should log `BMI088 answered` with `acc=0x18 gyro=0x68`.

Source: `microduck/tof/src/imu.rs`, `tof/src/main.rs`, `deploy/robotd.toml` `[head_imu]`, `docs/project/tof-on-demand.md`, `kinematics/src/head.rs`, `bmi088-rs` v0.1.2 `src/lib.rs`, HAT `sensors.kicad_sch`, `microduck_rl/.../robot_groundcontact.xml`, BMI088 datasheet tables 10 and 14 and section 9.2, the module's listing photos.

### 6.3 Reference: the shipped `imu_to_dxl` v2

Pollen's robot carries the same LSM6DSV16X on a small unpublished board with a microcontroller that emulates a Dynamixel Protocol 2.0 device at ID 200. It hangs off the servo bus on the same 3-pin connectors, powers from the bus, and answers the tick's sync read with the 12-byte block above (its full block is 20 bytes; the host reads 12). The MCU, firmware and PCB are not published, and the rework does not need them: the data contract is fully described by the host decoder, and the bridge carries the sensor instead.

Source: `microduck/docs/design/robotd-design.md` §1.1, `duck-control/src/imu.rs`.

### 6.4 Reference: the shipped head IMU

The firmware calls it the head module's BMI088 and reads it on the same I2C3 bus as the ToF, through the HAT's Qwiic port, at accelerometer 0x19 and gyroscope 0x68. The HAT's `sensors.kicad_sch` carries a BMI088 with VDD and VDDIO on 3.3 V, PS set for I2C, CSB1 and CSB2 high and all four interrupt pins unconnected. The head module itself is not published. Support arrived in firmware 0.11.0, off by default since 0.12.0 after measuring its CPU cost, and no daemon on the robot consumes the stream yet.

Source: `microduck/tof/src/imu.rs`, `docs/project/tof-on-demand.md`, HAT `sensors.kicad_sch`.

## 7. Sound

### 7.1 Design

Two I2S breakouts in the head on the RK3576's SAI2, with no codec chip and no codec driver.

- Playback: a MAX98357A class-D amplifier breakout (Adafruit 3006 or equivalent) driving a 3 W speaker. Inputs are bit clock, frame clock and data; it needs no MCLK and no control bus. Supply 2.5 to 5.5 V; use 5 V from the head buck for full output. The gain pin sets 3 to 15 dB in steps; the SD_MODE pin selects shutdown, left, right or (left + right) / 2 mono. Leave the breakout's default mono mix so stereo files in the voice bank play correctly.
- Capture: an INMP441 I2S MEMS microphone breakout. Inputs are bit clock and frame clock; it outputs 24-bit data in a 32-bit slot and needs no MCLK. Supply 3.3 V from the header. Tie the L/R pin for the left slot.

Wiring to the NANO-A header:

| Signal | Header pin | RK3576 | MAX98357A | INMP441 |
|---|---|---|---|---|
| bit clock | 12 | SAI2_SCLK | BCLK | SCK |
| frame clock | 35 | SAI2_LRCK | LRC | WS |
| data out | 40 | SAI2_SDO | DIN | |
| data in | 38 | SAI2_SDI | | SD |
| 5 V | 2 or 4 | | VIN | |
| 3.3 V | 1 or 17 | | | VDD |

Kernel: an overlay enabling `sai2` on its M0 pins and a sound card with the SAI as bit and frame master and Rockchip's dummy codec, which is built into the vendor kernel, as the codec. Run the card at a fixed 48 kHz with 32-bit slots, which suits both parts. `robotd` plays through `aplay` and the petting classifier captures 16 kHz mono through `arecord`, both via a `plughw` device, so ALSA converts rate and width. Volume is an ALSA `softvol` control in place of the codec's hardware mixer.

Firmware changes: `audio.device` in `robotd.toml` names the new card, the `aic3104-init` mixer script is replaced by the `softvol` definition and one or two `amixer` lines, and the `aic3x` DKMS package is dropped from `setup-board.sh`. Nothing in `robotd/src/sound.rs` or `pet-detect` changes.

Two things to expect:

- The microphone moves from the trunk to the head. The petting classifier was trained on recordings from the HAT's microphone in the trunk, listening for head scratches. Expect to retune its thresholds or retrain it on head-mounted recordings.
- The head is also where the speaker is. Place the microphone away from the speaker and the servo, and use the classifier's ambient-level gate as intended.

Source: MAX98357A and INMP441 datasheets, Radxa kernel defconfig (`CONFIG_SND_SOC_ROCKCHIP_SAI`, `CONFIG_SND_SOC_DUMMY_CODEC`, `CONFIG_SND_SIMPLE_CARD`), `microduck/robotd/src/sound.rs`, `pet-detect/src/lib.rs`, `deploy/robotd.toml` `[audio]`.

### 7.2 Alternatives considered

- ES8388 codec module plus a PAM8403 amplifier. Radxa's own CM4 IO board uses an ES8388 on SAI1 with the in-kernel `es8323` driver, so a worked device tree exists. More parts, an electret microphone with bias, and MCLK from pin 36, in exchange for a hardware mixer.
- USB sound dongle plus amplifier. No device tree work, but it takes the NANO-A's only USB-A port and adds bulk in the head. The fallback if the SAI2 overlay proves troublesome.
- Carrying the HAT's TLV320AIC3104 section across. Viable, the HAT's I2S wiring matches SAI2 on the Pi pins exactly, but it means a custom board and the DKMS module, both of which this revision removes.

### 7.3 Reference: the shipped HAT audio

Codec TI TLV320AIC3104 at I2C address 0x18 with a 12 MHz crystal for MCLK; Diodes PAM8406D stereo class-D amplifier on 5 V, one channel to a speaker labelled "5W LS" on a Wago connector; one MEMS microphone (LMA2718) on the codec's MIC2R input, exposed as `Mic3R`. Control on I2C3, data on the RK3566's I2S3 as bit and frame master through the `aic3104-i2c3` overlay, driver `snd-soc-tlv320aic3x` as a DKMS module, mixer levels set at boot by `aic3104-init.service`. `robotd` plays on `plughw:aic3104` and captures on `plughw:aic3104,0`.

Source: `microduck/deploy/audio/*.dts`, `aic3104-init.sh`, `aic3x-dkms/`, HAT schematic sheet 6/6.

## 8. Camera

Same sensor, shorter path.

- Sensor: Sony IMX219, the Raspberry Pi Camera Module v2 class. Pollen's store calls it a wide-angle camera and the ideas doc assumes about 62° horizontal FOV, so it may be a wide-lens IMX219 module rather than the stock Pi lens. Radxa's 8M 219 module or a Pi Camera Module v2 both fit. The RL model places `head_camera` in the head next to the ToF, which is now also where the compute module is.
- Physical: the NANO-A's 15-pin, 1.0 mm pitch CSI connector, wired as the Pi CAM0 port. On the Radxa CM4 that is the RK3576's CSI1 receiver with two lanes, enough for the IMX219 at 1080p30. A standard 15-pin camera cable of a few centimetres replaces the 22-pin to 15-pin adapter and the neck run the Zero 3W design needed.
- Device tree: an IMX219 node on CSI1 (`csi2_dphy1`, `mipi1_csi2`, `rkcif_mipi_lvds1`, `rkisp_vir1`) with its control I2C on the RK3576's I2C6 on M3 pins, and the camera enable on GPIO2_C5, the Pi CAM_GPIO position. The NANO-A schematic routes the connector's SCL and SDA to the CM4 ID_SC and ID_SD pins, so the bus is I2C6 and not I2C0 as earlier revisions said; I2C0 M1 is the SDA0/SCL0 pair that Pi carriers give to CAM1 and to the CM4 IO board's fan controller and RTC. Radxa's overlay for its CM4 on a Pi CM4 IO board with the camera on CAM0 therefore applies to the NANO-A unchanged, and is carried in `scripts/compute/camera/` and installed by `setup-camera.sh` as an Armbian user overlay (`camera-setup.md`). Without the overlay there is no `/dev/video*` and nothing in dmesg, which looks exactly like an unplugged camera.
- Capture: Rockchip rkisp on the vendor kernel, as today. The capture node is found by card name (node numbers change between boots). `mediad` pins the sensor mode with `media-ctl` at startup and the ISP scales to 720p. Expect the RK3576's rkisp to expose the same media graph shape as the RK3566's with different entity names; `mediad`'s discovery by name is the part to check.
- Encode: `mpph264enc` through `/dev/mpp_service` (Rockchip MPP), not V4L2 M2M. `/dev/mpp_service` and `/dev/rga` need a udev rule for the `video` group, and `mediad.service` needs `SupplementaryGroups=video`. The RK3576's MPP is the same API.
- 3A: Rockchip's `rkaiq_3A_server` with an IMX219 IQ file. Radxa's RK3576 images carry the rkaiq build and IQ files for the RK3576's ISP; `setup-rkaiq.sh` needs to point at those rather than the RK3566 ones. Its auto-exposure fires once, so `mediad` runs its own exposure loop.
- Field of view: on this kernel's vendor driver the 1920x1080 mode is a native-pixel centre crop with binning off, about 39° horizontal with the 62° lens, not the full-width readout the upstream `camera.rs` describes for the Zero 3W. Verified 22/09/2026. The upstream nominal intrinsics (62°, `fx` about 1062 at 1280 wide) are therefore wrong for this board unless the pipeline is changed. Options, verified and unverified, are in `camera-setup.md` section 4a: full sensor mode at 21 fps with an ISP 16:9 crop (works today), a binned 1640x1232 mode added to the driver (kernel rebuild, the proper fix), or a wider lens.
- Mounting: the sensor is mounted rotated (the docs disagree on a quarter turn or upside down). Nothing rotates in the pipeline; the viewer rotates. `videoflip` was measured to throttle the RK3566 to 408 MHz; do not assume the RK3576 is different without measuring.
- Heat: the compute module now shares the head with the sensor. Keep the module's heatsink and airflow away from the camera; sensor noise rises with temperature.

Reference, shipped robot: the Zero 3W's 22-pin CSI with the `radxa-zero3-rpi-camera-v2` overlay, which Armbian ships without the `rk3568-` prefix so `setup-board.sh` copies and renames it.

Source: `microduck/docs/project/media-bringup.md`, `scripts/setup-board.sh` (`configure_camera`), `scripts/setup-rkaiq.sh`, `mediad/src/pipeline.rs`, `mediad/src/exposure.rs`, Radxa CM4 schematic (CSI1 on the Pi CAM0 pins), CM4-NANO-A schematic (connector nets), Radxa overlays `radxa-cm4-rpi-cm4-io-radxa-camera-8m-219-cam0.dts` and `-cam1.dts` in `radxa-pkg/radxa-overlays`.

## 9. Time-of-flight sensor

Same sensor, now a few centimetres from the compute module.

Wiring: SDA to header pin 3, SCL to pin 5, 3.3 V from pin 1, and ground. Neither the carrier nor the Radxa CM4 pulls these pins up. The ToF breakout's TXS0104E level shifter does, through its internal 10 kΩ resistors, so no header resistors are fitted while it is on the bus. Without the ToF breakout the bus has no pull-ups unless the BMI088 breakout carries its own: add 4.7 kΩ from each line to 3.3 V in that case. If both breakouts carry pull-ups, measure the effective value and keep it in the 2.2 k to 4.7 k range at 400 kHz. The `microduck-nano-a-i2c8` overlay enables `i2c8` on its M1 pins at 400 kHz, and `setup-head-i2c.sh` installs it with the udev rule that points `/dev/i2c-pihat` at the RK3576 I2C8 controller, so `tofd` needs no change. On the board the bus came up as `/dev/i2c-8` (24/09/2026). With the codec gone the bus carries the ToF at 0x29 and the BMI088 at 0x18 and 0x68 (section 6.2). The sensor takes 3.3 V IO on the stock breakouts.

Reference and unchanged details:

- Sensor: ST VL53L8CX (the daemon also accepts a VL53L5CX and probes the device ID). 8 x 8 zones, 45° x 45° field of view, 15 Hz. In the head next to the camera (`tof` site in the RL model). A stock breakout with a Qwiic or STEMMA QT connector. Not every duck has one; `tofd` tolerates its absence.
- Bus on the shipped robot: hardware I2C3, shared with the audio codec, through the HAT's Qwiic J5 (JST SH 1.0 mm, 3V3) and a cable to the head. The HAT provides the only pull-ups, one 10 k pair (R12/R13). The overlay comment warns that head-cable capacitance can stretch rise times and suggests 200 kHz if 400 kHz proves flaky. With the sensor now beside the compute module that concern goes away.
- Address: 0x29 default; 0x52 also tried because a prototype once moved it.
- Device path: `/dev/i2c-pihat`, a udev symlink following the controller, with a numbered fallback.
- Access: `tofd` runs unprivileged as user `tofd` in the `i2c` group and uses `I2C_RDWR` ioctls in 2 KiB chunks.
- Bring-up cost: about 90 KB of firmware uploaded at every sensor start, a few seconds at 400 kHz. This is why the ToF is its own daemon.

Source: `microduck/tof/src/main.rs`, `sensor.rs`, `vendor/platform.c`, `systemd/tofd.service`, `deploy/audio/i2c3-pihat.dts`, `kinematics/src/tof.rs`.

## 10. Power: 6S LiPo

Decision: a 6S LiPo replaces the NP-F550. One buck in the head makes 5 V for everything that is not a servo.

Voltage fit.

| | Volts |
|---|---|
| 6S fully charged (4.20 V/cell) | 25.2 |
| 6S nominal (3.70 V/cell) | 22.2 |
| 6S empty under load (3.50 V/cell) | 21.0 |
| J288 range, recommended | 6.4 to 25.2, recommended 25.2 |

- The J288's recommended 25.2 V is exactly a full 6S, so the servos run at their design point and deliver the 35 rad/s rating rather than 16.5 rad/s at 12 V. No-load speed scales with supply, to roughly 29 rad/s at 21 V, so the RL actuator model needs pack voltage as a parameter. Standard 4.20 V/cell packs only: an HV LiPo (4.35 V/cell, 26.1 V) exceeds the servo's maximum. The S288 is ruled out at 12.6 V.
- No headroom above full charge. The J288 fault table has transient (0x02) and sustained (0x04) over-voltage codes, and fifteen servos braking together can pump the rail above pack voltage momentarily. A LiPo absorbs regeneration well, but a pack straight off the charger has nowhere to put it. The bridge reads the fault flags; a TVS or a small bulk capacitor bank at the distribution board is cheap insurance; charging to 4.15 V/cell removes the issue at little cost if it appears.

The head buck.

- Spec: input rated comfortably above 25.2 V (30 V or more, so a full pack plus regeneration transients is inside the rating), 5 V output at 3 A continuous, synchronous, low ripple, and an enable pin. The enable pin is the robot's power switch: a small switch or a latch on the enable line turns off the compute, the audio and the bridge together while the servo rail stays unswitched behind its own fuse. A module with an XT30 or screw input and a 5 V output header is fine for the prototype.
- Loads: the CM4 module and carrier (budget 2.5 A peak), the MAX98357A (up to 0.6 A at full output), the bridge over the neck (under 0.2 A). 3 A continuous is comfortable.
- Feeds: NANO-A header pins 2 and 4, the amplifier's VIN, and a 5 V and ground pair down the neck to the WeAct board's 5 V input, whose LDO makes the bridge's 3.3 V and powers the IMU and bus buffers.
- Input: its own +BATT and ground pair from the distribution board, fused, with bulk capacitance at the buck's input, since the run up the neck sits beside servo power and the 6S rail carries the servos' switching noise. Keep the buck's ground return to the carrier short.
- Why the head: the compute module is the largest 5 V load and it is in the head. Running +BATT up the neck at under half an amp needs a thinner conductor than 5 V at 2 to 3 A would, and the voltage drop across the neck harness no longer matters. The bridge's 0.2 A back down the neck at 5 V is small enough that a second buck in the trunk is not worth its parts; if the WeAct board's LDO runs warm, a small trunk buck for the bridge is a one-part change.

Current and distribution.

- Unitree's no-load figure is 0.45 A at 25.2 V per servo, measured spinning unloaded, so it is not the idle draw. The maximum line current is 1.92 A per servo, which puts fifteen servos at their limit together near 29 A. That is a ceiling, not an expected load. Standing and walking draw need measuring on the robot before pack capacity and fuse are fixed.
- Servo power does not go through the bridge board or the carrier. Use a distribution board (XT30 or XT60 in, main fuse, branches out): left leg, right leg, neck and head, and the buck feed. The head servo branch runs up the neck harness.
- The servo pigtail is PH 2.0 (SIGNAL, VCC, GND), about 2 A per contact. Branch VCC and GND from the distribution board per servo or per limb and share only the signal wire per bus segment.
- The bridge measures pack voltage with its own divider into a G474 ADC channel. The J288 reports supply voltage in 0.5 V steps, only eight or nine steps across a 6S discharge, too coarse for a gauge or a shutdown decision.

Firmware constants that change.

- `BATTERY_FULL_V` and `BATTERY_EMPTY_V` in `duck-control/src/model.rs` are 8.2 and 6.6 for the 2S pack. For 6S they become about 25.2 and 21.0. The `battery_empty_shutdown` sit-and-power-off behaviour keys off the same floor and matters more with a bare LiPo than with the protected NP-F550.
- `nominal_voltage` in `robotd.toml` (7.4, used by the optional `voltage_adapt` scaling) becomes 22.2 if that feature is used.
- Volts come from the bridge's ADC in the state frame instead of being averaged from servo registers.

Pack safety and mass.

- The NP-F550 carried its own protection circuit. A bare LiPo does not, so the distribution board needs a fuse sized for the measured peak, and the firmware low-voltage shutdown is the only cell protection in the loop. A balance connector must stay reachable for charging.
- The chosen 6S pack weighs 132 g (measured 30/09/2026) against about 100 g for the NP-F550, so the battery adds about 32 g to the trunk, on top of the 315 g the fifteen J288s add over the XL330s (39 g against 18 g each). Both numbers go into the RL project's robot model, as does the head (section 11.1). Together they take the robot from 0.74 kg to roughly 1.2 kg with no more peak joint torque, which makes mass the main lever on torque margin (section 12).

Reference, shipped robot: removable Sony NP-F550 type 2S Li-ion, firmware maps 8.2 V to full and 6.6 V under load to empty and shuts down at 6.6 V, about one hour of run time, servo rail straight from the battery through the HAT, no fuel gauge or ADC (voltage read from the servos' own supply register). The HAT's AP63205 buck and LM5050-1 ideal diode made 5 V from 5 to 28 V in.

Source: `microduck/duck-control/src/model.rs`, `deploy/robotd.toml`, HAT schematic sheet 3/6, Pollen store page, Unitree product page.

## 11. Physical layout and harness

No board of our own in this revision. Everything is a purchasable module or breakout, wired point to point.

### 11.1 Head

Contents: the head buck, the CM4 module on the NANO-A with a heatsink, the IMX219 on a short cable, the VL53L8CX, the BMI088 breakout at the `head_imu` site, the MAX98357A and speaker, the INMP441, the eye LED, the head fan and its MOSFET, the Wi-Fi antenna, and the head and mouth servos.

- Mass. The RL model's head body is 0.189 kg and the whole robot 0.737 kg with XL330s. Module, carrier, buck, heatsink, amplifier, speaker and microphone add roughly 80 to 120 g to the head, a 40 to 60 percent increase in what the neck servos carry and a shift in the whole-body centre of mass. The J288's peak driving torque is only modestly higher than the XL330's (section 5.1.1), so this weight eats most of the torque margin; the MJCF needs the new head and neck masses and inertias before any policy is trained for this hardware. Weigh the assembled head and put the number in the model.
- Heat. Budget 5 to 8 W of dissipation from the module under perception load, in a shell that also holds the camera. Fit a heatsink to the RK3576 from the start and give the shell a vent path. The head fan (`express-setup.md`) blows across it, switched by SoC temperature. A heatsink covering the whole module hides its maskrom button, which is why the NANO-A's BOOT switch matters.
- Antenna. Route the module's IPEX lead to an antenna against the shell top, away from the servo and the buck.
- Volume. Check the head CAD for a 55 x 40 mm board stack about 20 mm tall including heatsink, plus the buck module, before committing.

### 11.2 Trunk

Contents: the battery, the distribution board, the WeAct G474 bridge with its bus buffers and pull-ups, the LSM6DSV16X breakout at the `imu` site, and the ten leg servos with their branch wiring.

- Mount the IMU breakout rigidly at (-0.021, 0.000, -0.0147) m in the trunk frame and note its orientation for the mount quaternion (section 12).
- Keep the bridge's SWD or USB-C reachable for firmware updates without disassembly, or bring them to a small service port.

### 11.3 Neck harness

Conductors, trunk to head:

| Purpose | Conductors | Notes |
|---|---|---|
| Head servo branch | +BATT, GND | up to a few amps peak across five servos; size for the measured draw |
| Buck feed | +BATT, GND | under 0.5 A; separate from the servo branch so servo noise does not enter the buck unfiltered |
| Servo bus segment C | data | single wire, 6 Mbps, keep short and away from the power pairs where possible |
| Bridge link | TX, RX, GND | twisted pairs with ground, 2 Mbps to start |
| Bridge supply | 5 V, GND | under 0.2 A |

Nine to eleven conductors through four neck and head joints. Use a flexible silicone-insulated bundle, service loops at each joint, and one connector at each end (JST GH or similar) so the head detaches. The neck servo pigtails join the head branch inside the neck.

### 11.4 A future board

If a board is ever made, it merges the bridge, its buffers and pull-ups, the pack-voltage divider and the IMU onto one trunk PCB with PH 2.0 servo connectors per segment. The head side stays off the shelf: the NANO-A already does what a custom carrier would.

A KiCad footprint and symbol for the WeAct board, generated from WeAct's outline drawing, are in `hardware/kicad/` (see its README), so a carrier or breakout for the bridge can start without redrawing the module.

## 12. Calibration and risks

- Head mass and inertia in the RL model. Section 11.1. Retrain before expecting a policy to stand.
- Trunk IMU mount quaternion. Derived on the bench with the robot upright and tilted about known axes, then stored in `robotd.toml`. Wrong, it walks badly without a fault.
- Head IMU orientation. The MJCF `head_imu` site is what places its samples; if the breakout is mounted differently, update the site quaternion or every consumer gets rotated data with no fault raised. Nothing in the control loop depends on it, so this only affects mapping and vision work.
- Head IMU addresses. The driver looks only at 0x19 and 0x68, and the module's single SDO pin can only give 0x18 with 0x68 or 0x19 with 0x69. Until the driver takes addresses, the chip is silently absent. Section 6.2.
- Joint zero offsets and directions. The J288 reports a multi-turn rotor position relative to power-up plus a 13-bit absolute output-side encoder. Each joint needs a zero offset and a sign that map to the RL model's `DEFAULT_POSITION` frame. Store per-joint calibration on the bridge or in `robotd.toml`, and make the bench procedure repeatable.
- Stiffness. The XL330 P gain of 200 and its scaled variants are what the shipped policies were trained against through the BAM actuator model. J288 kp and kd are new parameters and the RL project will re-fit its actuator model to the J288; do not expect the shipped policies to transfer. The J288 is torque controlled with a voltage-limited speed, so the fit needs a torque-controlled BAM actuator rather than new parameters for the XL330's voltage-controlled one. The RL project's backlash variant assumes the XL330 arrangement, with the encoder on the output side of the play. On the J288 the loop closes on the rotor, so that assumption has to be reversed.
- Torque margin. The J288 delivers about 0.55 to 0.6 N·m at the output, about 40% above the XL330 the policies were trained against, and the robot gets about 60% heavier. Running the shipped policies in simulation on a 1.2 kg model: standing, turning and normal walking stay well under the J288's limit, and top-speed walking (0.54 N·m at the knee) stays just under it; sitting and standing still peak at 0.85 N·m at the knee, beyond it. The XL330's torque falls away with speed and the J288's does not, so its margin in fast motion is larger than these static figures suggest. `j288-testing.md` sections 12 and 15.
- Servo heating. At stall the windings heat about 1 °C/s at 0.5 N·m of motor torque and 4.5 °C/s at the 1.0 N·m clamp. Where the temperature settles under a sustained standing load is not yet measured. `robotd` needs winding-temperature thresholds for the J288 (Unitree shuts down at 120 °C). Mounting the aluminium-cased servos to metal will help at steady state.
- Thermal throttling of the RK3576 in the head. Watch clock frequency under the full perception load with the shell closed.
- The bridge link through the neck at 4 Mbps. 2 Mbps is the design rate until a scope says otherwise.
- HDSEL pull-up sizing on each five-servo segment. Measure the rising edge; fall back to the buffered drive.
- LPUART1 as a 6 Mbps single-wire bus master for segment C. The peripheral has the modes and the divider is exact, but Unitree's example uses a full USART. Section 5.8 has the fallback.
- Servo bus ground reference. The bridge's ground comes down the neck from the head buck; without the strap to the distribution board (section 5.8) the data lines are referenced through that loop.
- The J288 signal logic level is unconfirmed. Buffer or use five-volt-tolerant pins until it is.
- I2C pull-ups on I2C8 are the builder's to add, and two breakouts may already carry some.
- The petting classifier's microphone has moved. Retune or retrain.
- Camera IQ file and rkaiq build for the RK3576 ISP. `setup-rkaiq.sh` currently assumes the RK3566's.
- Supply noise. The module, IMU and camera sit on a buck fed from the servo pack. Camera and IMU noise are the symptoms to watch; input bulk capacitance and grounding are the fixes.
- Over-voltage at full charge (section 10) and the J288's 6.4 V floor are the two power edges; only the first one is near.

## 13. Open items to verify on hardware

- The J288 SIGNAL logic level and idle state, with a scope.
- J288 behaviour still open after the first bench tests (`j288-testing.md` section 14): winding temperature under a sustained standing load, the back-driven holding torque, the internal PD loop rate, the torque-speed envelope at 6S, and gear play measured at rest.
- Standing and walking current of fifteen J288 on 6S, to size pack, fuse, distribution and neck wiring, and whether over-voltage faults appear at full charge.
- The eMMC size of the ordered CM4 variant.
- The NANO-A's 5 V path from header pins 2 and 4 to the module, and whether the header I2C pins have pull-ups.
- Done 03/10/2026: the BMI088 module on I2C8 shows 0x18 (chip ID 0x1E), 0x29 and 0x68 (chip ID 0x0F) after moving the PS link to IIC; `tofd --imu` detects it and streams at 100 Hz. The module carries a 3.3 kΩ resistor network (marked 332), most likely the SDA and SCL pull-ups.
- The BMI088's mounted orientation against the MJCF `head_imu` site, checked by tilting the head and reading `head_imu.stream`.
- That the vendor kernel clocks UART7 to 4 Mbps, and that 4 Mbps survives the neck harness. Otherwise 2 Mbps.
- Boot into maskrom through the NANO-A BOOT switch and flash over its USB-C.
- Overlays for `uart7`, `i2c8` and `sai2` with the dummy codec on the chosen image. The IMX219 overlay is verified on hardware (22/09/2026): sensor probed on I2C6 and 720p NV12 streamed from the ISP main path, see `camera-setup.md`. Rockchip's 3A engine with a generated RK3576 tuning file for the IMX219 runs as `microduck-rkaiq.service` and the ISP output is auto exposed and colour corrected (`camera-setup.md` section 6); it needs a scheduling shim because nothing under systemd on this image may create a real-time thread, which the servo control loop will also meet.
- Rising-edge time on each servo segment with the chosen pull-up.
- Segment C on LPUART1 at 6 Mbps single-wire, with a scope, before the neck harness is built around it.
- The regulator on the LSM6DSV16X module: identify the SOT-23-5 part and measure its output with VCC at 3.3 V before wiring the SPI lines.
- WeAct solder bridges as delivered: SB3, SB5, SB6 and SB7 open, the state of SB4 (VBUS sense on PB2) and SB10.
- Done 03/10/2026: the WeAct board is the QFN48 variant (STM32G474CEU6); USB DFU flashing from `src/bridge/flash.sh` works and the blue LED on PC6 blinks. After a DFU flash the board needs a reset before its USB CDC port enumerates.
- Done 03/10/2026: the trunk IMU on SPI3 with INT1 on PA8 reads WHO_AM_I 0x70 and streams SFLP game rotation and gyro at 120 Hz through the FIFO (section 6.1).
- Done 03/10/2026: trunk IMU at rest for three minutes. The SFLP gyro bias estimate matches the measured gyro offset to within 5 mdps, yaw drifts 0.08°/min and roll and pitch under 0.07°/min (section 6.1).
- The IMU mount quaternion and the fifteen joint zero offsets, on the assembled robot.
- Head mass as built, and RK3576 temperature and clocks in the closed head under load.
- Which IMX219 module and lens is fitted.

## 14. References

- microduck firmware: https://github.com/pollen-robotics/microduck (local copy in `../microduck`)
- microduck RL: https://github.com/pollen-robotics/microduck_rl (local copy in `../microduck_rl`)
- RPI Robot HAT (KiCad, BOM, schematic PDF), reference only: https://github.com/pollen-robotics/elec_RPI_Robot_HAT
- rustypot (the shipped Dynamixel client, for reference): https://github.com/pollen-robotics/rustypot
- Radxa CM4 docs: https://docs.radxa.com/en/som/cm/cm4
- Radxa CM4 product brief: https://dl.radxa.com/cm4/docs/radxa_cm4_product_brief.pdf
- Radxa CM4 schematic v1.20 (connector pinout, pinmux table, boot mode config): https://dl.radxa.com/cm4/docs/hw/radxa_cm4_schematic_v1.20.pdf
- Radxa CM4 maskrom guide: https://docs.radxa.com/en/som/cm/cm4/low-dev/rkdevtool_maskrom
- Radxa kernel, Radxa CM4 device trees (`rk3576-radxa-cm4-io.dts`, `rk3576-radxa-cm4-rpi-cm4-io.dts`): https://github.com/radxa/kernel/tree/linux-6.1-stan-rkr5.1/arch/arm64/boot/dts/rockchip
- WeAct STM32G474 core board (QFN48) schematic v1.0 and outline: `docs/datasheets/WeAct-STM32G474CoreBoard_V10_SchDoc.pdf`, `docs/datasheets/WeAct-STM32G474CoreBoard_V10 Board Shape 外形.pdf`
- WeAct STM32G474 core board repository (schematics, outline drawings, STEP models, ME6239 and ST datasheets): https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard
- KiCad footprint and symbol for the WeAct board (generated): `hardware/kicad/`, `scripts/kicad/gen_weact_kicad.py`
- STM32G474 datasheet DS12288 (pin definitions and alternate functions): https://www.st.com/resource/en/datasheet/stm32g474ce.pdf
- STM32G474CE pin and alternate-function data (embassy stm32-data): https://github.com/embassy-rs/stm32-data-generated/blob/main/data/chips/STM32G474CE.json
- LSM6DSV16X module pinout photo: `docs/datasheets/lsm6dsv16xtr.png`
- Radxa kernel defconfig (SAI, dummy codec, simple card, USB audio): https://github.com/radxa/kernel/blob/linux-6.1-stan-rkr5.1/arch/arm64/configs/rockchip_linux_defconfig
- Armbian board config for the Radxa CM4 IO: https://github.com/armbian/build/blob/main/config/boards/radxa-cm4-io.conf
- Pi CM4 and Radxa compute module pinout sheets: https://github.com/mi4code/cm-compare
- Waveshare CM4-NANO-A: https://www.waveshare.com/wiki/CM4-NANO-A
- Waveshare CM4-NANO-B (the tested sibling): https://www.waveshare.com/wiki/CM4-NANO-B
- WeAct STM32G474 core board (schematic, board shape, STEP): https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard
- STM32G474 reference manual RM0440 (USART half-duplex mode, baud generation): https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- SparkFun 6DoF IMU Breakout LSM6DSV16X hookup guide (SPI jumper): https://docs.sparkfun.com/SparkFun_6DoF_LSM6DSV16X/hardware_overview/
- ST LSM6DSV16X datasheet: https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf
- Bosch BMI088 datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
- bmi088-rs driver used by `tofd` (addresses, defaults, Madgwick fusion): https://github.com/pollen-robotics/bmi088-rs
- BMI088 module as ordered (I2C variant, 7-pin header, single SDO): https://www.aliexpress.com/item/1005013089348960.html
- Adafruit MAX98357A I2S amplifier breakout: https://www.adafruit.com/product/3006
- Pollen Microduck product page (1 GB + 32 GB, NP-F550): https://store.pollen-robotics.com/products/microduck
- Unitree digital servo specifications: https://www.unitree.com/DigitalServo/
- Unitree J288/S288 support pages (quick start, control modes, configuration, protocol, fault table): https://support.unitree.com/home/en/J288-S288%20Servo/development_guide. The site renders with JavaScript; a plain HTTP fetch returns an empty page
- Unitree J288/S288 manual and STM32/Python notes: `docs/datasheets/` in this repo
- J288 bench test results: `docs/j288-testing.md`; tools: `scripts/j288/`
- Unitree digital servo examples (Python and STM32 protocol reference): https://github.com/unitreerobotics/digital_servo
- Rockchip RK3568 datasheet (UART "up to 4Mbps", shared limit with the RK3576 family): https://dl.radxa.com/rock3/docs/hw/datasheet/Rockchip-RK3568-Datasheet-V1.0-20201210.pdf
- RP2350 datasheet (the fallback bridge): https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- ROBOTIS Dynamixel Shield reference circuit (the HAT's TTL buffer): https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/

## 15. Changes from the 07/09/2026 revision

- Compute: Radxa Zero 3W in the trunk replaced by a Radxa CM4 on a Waveshare CM4-NANO-A in the head. Reason: perception headroom on the same Rockchip software family, with a purchasable carrier instead of a custom board.
- Bridge: RP2350 replaced by an STM32G474 on a WeAct core board. Reason: native single-wire half duplex, exact baud rates at 168 MHz, six UARTs.
- Link: UART2 at header pins 8 and 10 replaced by UART7 at pins 16 and 18. The console keeps UART0.
- HAT: dropped entirely, including the plan for a board derived from it. Audio, ToF port, buck and battery detection are all replaced by breakouts, a buck module and the bridge's ADC.
- Audio: MAX98357A and INMP441 on SAI2 with the dummy codec, in the head, in place of the HAT codec and its DKMS module.
- Camera: 15-pin CSI on the carrier, 2-lane, short cable, instead of the 22-pin connector and neck run.
- Power: one buck in the head instead of one in the trunk; +BATT runs up the neck, 5 V runs back down to the bridge.
- IMU: unchanged part, now explicitly a SparkFun breakout on SPI to the G474.

## 16. Changes from the 09/09/2026 revision

- Head IMU: a BMI088 is added in the head on I2C8 beside the ToF, at the RL model's `head_imu` site. Reason: firmware 0.11.0 and later reads it through `tofd` and serves `head_imu.stream`, and staying consistent with Pollen's build keeps the upstream mapping and vision work usable. Off by default. The one firmware change is configurable I2C addresses in `bmi088-rs`, because the generic module's single SDO pin cannot produce the HAT's 0x19 and 0x68 pair. Section 6.2.
- Section 6 restructured into trunk IMU, head IMU and the two shipped references. Section 9 and the header table note the shared bus and the pull-up budget.

## 17. Changes from the 15/09/2026 revision

- Bridge hookup: new section 5.8 with the G474 pin allocation, the per-segment bus wiring in both drive variants, the trunk IMU breakout pin map, power, and the board's programming and debug features, from the WeAct schematic and the G474 pin table.
- Bridge UART count corrected: the LQFP48 part has four usable UARTs (USART1, USART2, USART3, LPUART1), not six. Segment C goes on LPUART1 and the console moves to USB CDC or SWD.
- Bridge board: the part is the STM32G474CET6 (LQFP48) as marked on the WeAct schematic, and the LDO is an ME6239A33 rated 250 mA with 3.3 to 20 V in.
- Trunk IMU: the breakout is the generic LSM6DSV16X module on hand rather than the SparkFun board. Same signals; pin names and the regulator caution are in section 5.8.

## 18. Bridge board correction, 03/10/2026

- The WeAct board delivered is the QFN48 variant (STM32G474CEU6), not the LQFP48 one the 15/09/2026 revision was drawn from. Its schematic and outline replace the LQFP48 ones in `docs/datasheets/`, and the KiCad footprint and symbol in `hardware/kicad/` are regenerated for it.
- The QFN48 board brings out PC4, PC6, PC10 and PC11 on the four header positions that were not connected, moves the blue LED from PA8 to PC6, and adds a W25Q64 QSPI flash on PA6, PA7, PB0, PB1, PB10 and PB11.
- Pin allocation (section 5.8) changed to keep clear of the flash: the trunk IMU moves from SPI1 (PA5, PA6, PA7, CS PA4, INT1 PB5) to SPI3 (PC10, PC11, PB5, CS PA15, INT1 PA8); the pack voltage moves from PB0 to PA0; segment C's buffered-variant DE moves from PB1 to PB12. The three servo data pins and the compute link are unchanged.
- UART4 on PC10 and PC11 is available on this package and replaces USART3's PB10 and PB11 pins, now taken by the flash, as segment C's fallback.
- Solder bridge names follow the QFN48 schematic (SB3 and so on, formerly S3).
