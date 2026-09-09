# Microduck Unitree: hardware and hookup

The hardware for the reworked Microduck: Unitree J288 servos in place of the Dynamixel XL330s, a bridge microcontroller that owns the servo bus and the IMU, a 6S LiPo, and the rest of the Pollen alpha carried over. For each subsystem this document gives the design we are building and, as reference, how the shipped Pollen robot does it and where that is known from.

Date: 07/09/2026.

## 1. Sources and confidence

Pollen does not publish a wiring guide for the shipped robot. The reference material here is reconstructed from three kinds of source, and each section says which one it leans on.

- Firmware and provisioning code in the `microduck` repository (sibling folder `../microduck`). Device-tree overlays, udev rules, driver code and comments give the bus, address and pin for almost every part.
- The open HAT design, `pollen-robotics/elec_RPI_Robot_HAT` on GitHub (KiCad 9, rev C1, BOM and production schematic PDF). The robot's carrier board is this HAT or a close variant.
- Vendor documentation: Radxa Zero 3W docs, the Rockchip RK3566/RK3568 datasheet, the RK3568 pinctrl in the Linux tree, Unitree's J288 pages, and the J288 manual and notes in `docs/datasheets/`.

Two things about the shipped robot are not public: the `imu_to_dxl` board design, and the RAM and eMMC fitted to Pollen's development boards. Neither affects the rework.

## 2. Design decisions

| | Shipped Pollen alpha | This rework |
|---|---|---|
| Servos | 15 x Dynamixel XL330-M288-T, TTL bus at 1 Mbps | 15 x Unitree J288, single-wire bus at 6 Mbps |
| Servo bus master | RK3566 UART2 through the HAT's TTL buffer | Bridge MCU (RP2350 recommended) |
| SBC to servo link | Dynamixel Protocol 2.0 on `/dev/ttyS2` | Our own framed protocol, one round trip per tick, 4 Mbps UART |
| IMU | LSM6DSV16X on the `imu_to_dxl` board, emulating a Dynamixel device at ID 200 | LSM6DSV16X on the bridge MCU, same 12-byte data block delivered inside the state frame |
| Firmware seam | `DynamixelIo` implements `RobotIo` | New `BridgeIo` implements `RobotIo`; everything above it unchanged |
| Battery | NP-F550 2S Li-ion, 6.6 to 8.2 V | 6S LiPo, 21.0 to 25.2 V |
| SBC and MCU supply | HAT buck (AP63205) | Standalone 25 V to 5 V buck converter |
| Carrier board | RPI Robot HAT rev C1 | Prototype: no HAT. Bridge board, buck and ToF wired directly to the header. Final: one board derived from the HAT with the MCU, IMU, servo buffers and audio on it |
| Audio | HAT codec, amplifier, microphone | Not fitted on the prototype; returns on the final board |
| Camera, ToF, radios | as below | unchanged |

No Dynamixel emulation anywhere. The bridge speaks J288 frames to the servos and our protocol to the SBC. The reasons are in section 5.4.

## 3. System overview

Prototype configuration (no HAT):

```text
              6S LiPo (21.0 to 25.2 V)
                 |                        |
     servo power distribution       buck converter 25 V -> 5 V, 3 A
     board (fuse, XT30 in,                |
     per-limb branches)          +--------+--------+
                 |               |                 |
                 |          header pins 2/4    bridge MCU (RP2350) + LSM6DSV16X
                 |          Radxa Zero 3W          |  own protocol
                 |               |    UART2, pins 8/10, 4 Mbps, full duplex
                 |               +-----------------+
                 v                                 |
   15 x J288  <--- 6 Mbps single wire, buffered, PIO
   IDs 0..14

   Radxa Zero 3W header pins 3/5 (I2C3, with added 10 k pull-ups and 3V3) --- VL53L8CX ToF
   Radxa Zero 3W --- 22-pin MIPI CSI --- IMX219 camera
                 --- SDIO / UART HCI --- AIC8800D80 Wi-Fi 6 / BT 5.4 (on module)
   No audio on the prototype.
```

Final configuration: one board derived from the HAT carries the buck, codec, amplifier, microphone, Qwiic ToF port, the RP2350, the IMU and the 6 Mbps bus buffers, on the 40-pin header. Servo power stays on the separate distribution board. Section 11 has the plan.

## 4. Single-board computer

Unchanged from the shipped robot.

### 4.1 What it is

Radxa Zero 3W. Rockchip RK3566, four Cortex-A55 up to 1.6 GHz, Mali-G52-2EE, a 0.8 TOPS INT8 NPU, LPDDR4, optional eMMC, microSD, one USB 3.0 Type-C host and one USB 2.0 Type-C OTG, a 22-pin MIPI CSI connector and a Pi-compatible 40-pin header in a 65 x 30 mm Pi Zero footprint. Wireless is Radxa's Wireless Module D8 (AIC AIC8800D80, Wi-Fi 6 over SDIO, Bluetooth 5.4 over UART HCI). The BT radio's late attach and pairing quirks in the firmware docs trace to this chip.

Source: Radxa docs, `microduck/README.md`, `microduck/btd/systemd/btd.service`.

### 4.2 RAM and storage

Radxa sells the Zero 3W with 1, 2, 4 or 8 GB LPDDR4 and 0, 8, 16, 32 or 64 GB eMMC. microSD is always present and bootable. Pollen's product page lists the shipped Microduck as "1GB RAM + 32GB storage".

Recommendation: 4 GB RAM with 32 GB eMMC for development boards. 1 GB with 32 GB eMMC is the proven production floor.

- The shipped robot runs the full stack (ONNX Runtime at 50 Hz, GStreamer hardware H.264 and WebRTC, the NPU detector, five daemons) on 1 GB, so 1 GB is sufficient for the software as shipped.
- Development boards build the `aic3x` DKMS module, compile overlays and run benchmarks. 4 GB removes memory as a variable for a small price step.
- `/var/log` is a zram device on this image, so journald lives in RAM too.
- Storage must be eMMC, not only microSD. The updater's design assumes eMMC and keeps current, previous and golden releases plus a policy set, ONNX Runtime, the vendor kernel and GStreamer. 32 GB matches the product and leaves room for datasets and dev pushes.
- Choose the variant with the header populated. The HAT sits on it.

Source: Radxa product docs, Pollen store page, `microduck/deploy/README.md`, `microduck/docs/design/updater-design.md` §7.2.

### 4.3 Operating system

Armbian for Radxa Zero 3 (26.2.1 Minimal at time of writing), switched to the Armbian vendor kernel `linux-image-vendor-rk35xx` (Rockchip BSP 6.1) by `setup-board.sh`. The vendor kernel is required: the I2S3 clock tree for the codec, the rkisp MIPI-CSI driver, the MPP hardware encoder and the NPU driver exist only there. Overlays are named `rk3568-*.dtbo` and `overlay_prefix` must be `rk3568`.

The rework keeps `uart2-m0` (the bridge link) and `i2c3-pihat` (the ToF bus) and drops `aic3104-i2c3` and the DKMS codec module on the prototype, since there is no codec to drive. `setup-board.sh`'s audio step should be skipped or made conditional rather than left to fail soft on every provision.

Source: `microduck/docs/robot/install-dev.md`, `microduck/scripts/setup-board.sh`.

### 4.4 Header pins in use

Radxa Zero 3W header, RK3566 function, and what sits there. Verified against the HAT schematic, the Radxa header table and the RK3568 pinctrl.

| Header pin | RK3566 signal | HAT net | Shipped robot | Prototype (no HAT) | Final board |
|---|---|---|---|---|---|
| 1, 17 | +3.3 V out | +3V3 | HAT logic | ToF and pull-up supply | as shipped |
| 2, 4 | +5 V in | +5V | HAT buck powers the SBC | buck converter output | board buck |
| 3 | GPIO1_A0, I2C3_SDA_M0 | IO_02 | codec, BMI088, Qwiic J5 (ToF) | ToF SDA, add 10 k pull-up | codec, ToF |
| 5 | GPIO1_A1, I2C3_SCL_M0 | IO_03 | as above | ToF SCL, add 10 k pull-up | codec, ToF |
| 8 | GPIO0_D1, UART2_TX_M0 | IO_14 | Dynamixel bus TX via buffer | bridge RX, direct 3.3 V | same |
| 10 | GPIO0_D0, UART2_RX_M0 | IO_15 | Dynamixel bus RX via buffer | bridge TX, direct 3.3 V | same |
| 11 | GPIO3_A3, I2S3_SCLK_M0 | IO_17 (not used by HAT) | see I2S note, section 7 | unused | codec BCLK |
| 12 | GPIO3_A2, I2S3_MCLK_M0 | IO_18 BCLK | see I2S note, section 7 | unused | unused |
| 21, 24 | GPIO4_C5/C6, UART9_M1 TX/RX | IO_09/IO_08, Qwiic J7 | spare | spare second UART | spare |
| 27, 28 | GPIO4_B2/B3, I2C4_M0 | ID_SD/ID_SC | HAT EEPROM (not fitted) | spare second I2C | spare |
| 31 | GPIO3_B4 | IO_06 | battery-present detection | unused (bridge reports pack volts) | optional |
| 35 | GPIO3_A4, I2S3_LRCK_M0 | IO_19 WCLK | codec frame clock | unused | codec WCLK |
| 38 | GPIO3_A6, I2S3_SDI_M0 | IO_20 codec DOUT | microphone capture | unused | codec DOUT |
| 40 | GPIO3_A5, I2S3_SDO_M0 | IO_21 codec DIN | speaker playback | unused | codec DIN |

On this SBC only pins 3/5 (I2C3) and 27/28 (I2C4) are hardware I2C. UART9 on pins 21/24 is a spare full-duplex UART if a second link is ever wanted.

## 5. Servos and the bridge

### 5.1 The Unitree J288 bus

From the manual in `docs/datasheets/` and Unitree's product page.

| | Dynamixel XL330-M288-T (shipped) | Unitree J288 (rework) |
|---|---|---|
| Physical layer | TTL single-wire half duplex | TTL single-wire half duplex ("1-Wire", TTL multi-point bus) |
| Baud | 1 Mbps, configurable | 6 Mbps, fixed, 8N1 |
| Protocol | Dynamixel Protocol 2.0, register map, sync read/write | Fixed frames: 20-byte command (0xFE 0xEE header), 26-byte reply (0xFC 0xEE), CRC32 |
| Addressing | IDs 0 to 252, broadcast 254 | IDs 0 to 14, broadcast 15 (no reply). Maximum 15 servos per bus |
| Control | goal position, PID gains in registers | hybrid: tau = tau_ff + kp (p_des - p) + kd (w_des - w), rotor-side units, gear ratio 288.35 |
| Supply | 3.7 to 6 V rated (run at 2S) | 6.4 to 25.2 V, 25.2 V recommended |
| Feedback | position, velocity, current, voltage, temperature | rotor position, speed and torque, 13-bit output encoder, case and winding temperature, voltage in 0.5 V steps, fault flags |
| Connector | JST EH 3-pin | PH 2.0 3-pin: SIGNAL, VCC, GND |
| Size, mass | 20 x 34 x 26 mm, 18 g | 20 x 34 x 26 mm, 35 g |
| Stall torque | 0.52 N·m at 5 V | 1.5 N·m |
| Max speed | | 35 rad/s at 25.2 V, 16.5 rad/s at 12 V |

Fifteen servos fill the bus exactly. Nothing else can share it.

### 5.2 Why the RK3566 cannot be the bus master

The RK3566/RK3568 datasheet states the UART controllers reach "up to 4Mbps" with the right input clock. The J288 baud is fixed at 6 Mbps, so no overlay or clock setting puts the servos on an RK3566 UART. A bridge microcontroller is required. Unitree's own examples drive the bus from an STM32F413 with its USART in single-wire half-duplex mode, or from a PC through their single-bus-to-USB module.

### 5.3 Bridge MCU

Recommendation: RP2350 (Raspberry Pi Pico 2 for the prototype, an RP2350-Zero class module or the bare chip on the final board).

- Exact baud rates. The UART clock is 150 MHz with a fractional divider: 6 Mbps is 150 / (16 x 6) = 1.5625 = 1 + 36/64, and 4 Mbps is 2.34375 = 2 + 22/64. Both exact.
- PIO for the servo bus. A PIO state machine implements a 6 Mbps UART at 25 clocks per bit and handles the single-wire half duplex natively by flipping the pin direction. The two hardware UARTs (ARM PL011, no driver-enable output) then serve the SBC link and a debug console. Splitting the fifteen servos across two or three PIO buses is cheap if bus time ever matters.
- Two Cortex-M33 cores at 150 MHz with single-precision FPU. One core can own the servo bus and the other the SBC link and the IMU. The per-tick work (thirty frames, thirty CRC32s, one IMU FIFO read) is small.
- 3.3 V logic, not 5 V tolerant. Keep a buffer between the chip and the servo signal line until the J288's logic level is confirmed (section 5.6).
- RP2350 A2 silicon has the E9 erratum: internal pull-downs leak when a pin is an input. UART lines idle high with pull-ups, so it does not bite here, but do not rely on internal pull-downs anywhere on the board.
- Boards: Pico 2 (21 x 51 mm, both UARTs and all PIO available), Waveshare RP2350-Zero (18 x 23.5 mm), Pimoroni Tiny 2350, Seeed XIAO RP2350. SparkFun's Thing Plus RP2350 has a Qwiic connector that pairs with SparkFun's LSM6DSV16X Qwiic breakout for a no-solder IMU test.

Alternatives considered:

- Teensy 4.1 or 4.0 (i.MX RT1062, Cortex-M7 at 600 MHz, eight LPUARTs with hardware RTS direction, high-speed USB). Very capable, but 6.000 Mbps is not a clean fit: from the default 24 MHz UART clock it needs oversampling ratio 4 (the minimum, marginal noise margin), and from the 80 MHz PLL tap the nearest rate is 6.154 Mbps, a 2.6% error. Workable only after a bench test with a J288. The 4.1 is also as long as the SBC; the 4.0 is the size-appropriate sibling.
- ESP32-C3 and ESP32-S3. Excluded. Espressif documents the family's UARTs at up to 5 Mbps, the C3 has no FPU, and the radios are redundant on this robot.
- STM32 (F4, G4, H7) with a 96 or 192 MHz USART clock hits 6 Mbps exactly and is what Unitree uses. A sound alternative if the RP2350 is not wanted.

### 5.4 Protocol and firmware seam

The bridge speaks our own protocol to the SBC. No Dynamixel emulation.

Why:

- `duck-control` already has the right seam. The `RobotIo` trait (`duck-control/src/io.rs`) has `read()` returning joint positions, velocities, currents and IMU data, `write()` of joint targets, `set_torque`, `set_gain`, `slow_sensors()` for pack volts and per-joint temperatures, and `imu_stale` and `imu_ready`. `DynamixelIo` is one implementation and `FakeIo` another. Everything above the trait (observations, policy, safety, limp-fall, IPC, health, the update gate) never sees a bus packet. A `BridgeIo` implementation over a serial port is a few hundred lines of Rust.
- Emulating sixteen Dynamixel register maps on the MCU would force J288 semantics (multi-turn rotor position, hybrid control, torque feedback) through an XL330-shaped hole (single-turn ticks, a P gain, current), with Protocol 2.0 byte stuffing and strict reply ordering on top. Two protocol layers to debug instead of one.
- The J288's hybrid control (tau_ff, p_des, w_des, kp, kd) is available end to end for the RL side to use.

What is given up: `rustypot`'s scan and wizard tools, `robotd init`'s direct-bus path, and running Pollen's stock releases. The first two are small tools to rewrite; the third is gone the moment the servos change.

Frame sketch, one round trip per 50 Hz tick, fixed length, sequence number, CRC-16 or CRC-32. A specification, not this sketch, goes in a separate bridge-protocol document.

| Direction | Contents | Approximate size |
|---|---|---|
| SBC to bridge | seq, mode flags (torque on, per-joint enable), 15 x (p_des, w_des, kp, kd, tau_ff) in output-side SI units | about 300 bytes |
| Bridge to SBC | seq, 15 x (position, velocity, torque, temperature, fault bits), IMU block (12 bytes, section 6), pack voltage from the bridge's own ADC, bridge status | about 250 bytes |

At 4 Mbps that is about 1.4 ms of link time per tick, and the fifteen J288 exchanges cost the bridge roughly 1.2 to 1.5 ms of bus time at 6 Mbps. Well inside the 20 ms tick.

Firmware changes in `microduck` (a fork):

- `duck-control/src/bridge.rs` (new): `BridgeIo` implementing `RobotIo`, plus the `init` and `relax` operations `robotd init` needs (today those call `DynamixelIo` inherent methods).
- `duck-control/src/model.rs`: bus IDs 0 to 14, battery constants for 6S (section 10). Joint order, names and `DEFAULT_POSITION` stay; the RL contract depends on them.
- `duck-control/src/imu.rs`: unchanged if the bridge ships the same 12-byte block. The mount quaternion becomes a parameter.
- `robotd-params` and `deploy/robotd.toml`: `[bus]` gains the protocol and baud, and points at the new port.
- `set_gain` today writes an XL330 P gain of 200 and the standing and limp-fall modes scale it. For the J288 this becomes kp and kd parameters, and it is a retune, not a unit conversion.
- `currents_ma` in `Sensors` has no J288 equivalent. Report torque, or derive a current estimate, and check what consumes the field before deciding.

### 5.5 Link to the SBC

Prototype and final board alike: UART2 on header pins 8 (SBC TX, GPIO0_D1) and 10 (SBC RX, GPIO0_D0), wired straight to two RP2350 UART pins. Both sides are 3.3 V, full duplex, no buffer. The `uart2-m0` overlay, the kernel-console removal and the `serial-getty@ttyS2` mask from `setup-board.sh` carry over unchanged, so the port is still `/dev/ttyS2` and `robotd.toml`'s `[bus] port` does not move. With no HAT in the way there is nothing else on those pins.

Spare: UART9_M1 on pins 21 (TX, GPIO4_C5) and 24 (RX, GPIO4_C6) is a second full-duplex UART on the header, needing a `uart9m1` overlay. Not required by this design; noted in case a second link or a bridge console on the SBC side is ever wanted.

4 Mbps on the RK3566 is inside the datasheet limit but has not been exercised by the microduck repo (the shipped bus runs at 1 Mbps). If the vendor kernel will not clock UART2 that high, 2 Mbps still fits the tick comfortably (about 2.8 ms of link time).

### 5.6 Bus electrical

- Single-wire, 6 Mbps, a bit is 167 ns. Drive the bus through a fast tri-state buffer pair (the HAT's SN74LVC1G125 and SN74LVC1G126 are fine parts, with about 4 ns propagation) with direction from a PIO-controlled GPIO or by letting PIO drive the pin bidirectionally. Do not reuse the HAT's PNP auto-direction network, whose RC time constants were chosen for 1 Mbps.
- Signal level. The J288 manual does not state the SIGNAL logic level; Unitree's STM32F413 example is a 3.3 V part connected through an adapter board. Confirm with a scope before connecting a 3.3 V-only RP2350 pin directly. 74LVC inputs are 5 V tolerant, which is why the buffer is the safe default.
- One 10 k pull-up on the data line, series protection resistor and a clamp as on the HAT (150 R, 5.1 V zener).
- Topology: the data line is shared by all fifteen servos; power is not run through the servo pigtails (section 10). Splitting into two or three PIO buses shortens the star wiring to each limb and halves the bus time, and costs nothing on the RP2350.

### 5.7 Reference: the shipped Dynamixel bus

For context, since the HAT's Dynamixel section is what the prototype leaves unused.

- Port `/dev/ttyS2`, RK3566 UART2 M0 on header pins 8 and 10, `uart2-m0` overlay, kernel console removed from it and `serial-getty@ttyS2` masked. 1 Mbps, Dynamixel Protocol 2.0 via `rustypot` with `TIOCEXCL`.
- Sixteen devices: servos at IDs 10 to 14 (right leg), 20 to 24 (left leg), 30 to 34 (neck, head, mouth) and the `imu_to_dxl` board at ID 200. One `sync_read` of registers 124 to 135 across all sixteen and one `sync_write` of goal positions per tick, voltage and temperature (144 to 146) once a second. Startup pins `return_delay_time = 0`, `baud_rate = 3`, `pwm_slope = 255`, `shutdown = 52`.
- The HAT's half-duplex circuit follows the ROBOTIS Dynamixel Shield: direction derived from TX through a PNP (MMBT3906) and 10 k/20 k network, SN74LVC1G125 to the line, SN74LVC1G126 and a 74LVC1G08 back, 150 R series, 5.1 V zener, two JST EH 3-pin connectors with the battery rail through a 0 R link marked 3 A. An RS485 path (SIT3088E, two JST EH 4-pin) is present and unused.

Source: `microduck/duck-control/src/bus.rs`, `model.rs`, `deploy/robotd.toml`, `docs/design/robotd-design.md` §1.1, HAT schematic sheet 4/6.

## 6. IMU

### 6.1 Design

One ST LSM6DSV16X on the bridge board, read by the RP2350 over SPI (up to 10 MHz) or I2C, and delivered to the SBC inside every state frame. The SFLP engine on the chip does the fusion; neither the bridge nor the SBC runs a filter. A second IMU is not planned; the RL model's `head_imu` site and the HAT's dormant BMI088 are not used by the shipped firmware either.

Keep the data block the shipped firmware already decodes, so `duck-control/src/imu.rs` stays as is:

| Bytes | Contents |
|---|---|
| 0 to 5 | gyro x, y, z as little-endian i16 raw counts at ±500 dps (17.5 mdps/LSB) |
| 6 to 11 | SFLP game-rotation quaternion x, y, z as IEEE half floats; w = sqrt(1 - x² - y² - z²), positive |

Chip configuration, inferred from the host's assumptions: SFLP game rotation vector (no magnetometer), gyro full scale ±500 dps, SFLP output around 100 Hz (the host waits about 25 samples before trusting orientation), gyro bias estimation on. ST's FIFO emits the game rotation vector as exactly three fp16 values with w positive, so the format is a copy, not a conversion. Accelerometer full scale is unobservable from the host and free to choose.

Behaviour the host relies on, to reproduce on the bridge:

- All-zero quaternion bytes mean "SFLP has not started"; the host holds its last value. Send zeros until the first fused sample exists.
- The host flags the IMU as frozen after 25 identical blocks. Update the block from the sensor FIFO on the bridge at the sensor's rate and never block a reply on a sensor read.
- The host applies a fixed mount rotation (`SflpDecoder::DEFAULT_MOUNT`, +90° about Y for Pollen's placement). The bridge sends raw chip frame data; the mount quaternion for our board's position is derived on the bench and promoted to a `robotd.toml` parameter. This is the calibration that fails quietly: wrong, it produces a robot that walks badly rather than one that reports a fault.

The RL model puts the `imu` site at (-0.021, 0.000, -0.0147) m in the trunk frame. Mount the bridge board close to that.

Source: `microduck/duck-control/src/imu.rs`, `bus.rs`, `microduck_rl/.../robot_groundcontact.xml`.

### 6.2 Reference: the shipped `imu_to_dxl` v2

Pollen's robot carries the same LSM6DSV16X on a small unpublished board with a microcontroller that emulates a Dynamixel Protocol 2.0 device at ID 200. It hangs off the servo bus on the same 3-pin connectors, powers from the bus, and answers the tick's sync read with the 12-byte block above (its full block is 20 bytes; the host reads 12). The MCU, firmware and PCB are not published, and the rework does not need them: the data contract is fully described by the host decoder, and the bridge carries the sensor instead.

Source: `microduck/docs/design/robotd-design.md` §1.1, `duck-control/src/imu.rs`.

## 7. Sound

Not fitted on the prototype. The codec, amplifier and microphone all live on the HAT, and the HAT is dropped for now. The firmware tolerates this: `robotd.toml` `[audio] enabled = false` keeps the daemon quiet, and the design docs state that a board without the codec walks identically. The theremin, chorale and petting features are unavailable until audio returns.

Audio returns on the final board by carrying the HAT's audio section across unchanged (section 11.2), so the overlays, DKMS module and mixer script below apply to that board as they do to the shipped robot. Everything in 7.1 to 7.3 describes that design.

### 7.1 Components

- Codec: TI TLV320AIC3104 (U2 on the HAT), I2C address 0x18, 12 MHz crystal oscillator (Y1) feeding its MCLK, 1.8 V core from an XC6206P182 LDO.
- Amplifier: Diodes PAM8406D stereo class-D (U1), 5 V supply, fed from the codec's line outputs LEFT_LOP/LOM and RIGHT_LOP/LOM.
- Speaker: one loudspeaker on the Wago 2059 two-pole connector J1, driven by the amplifier's right channel (the left channel goes only to test points). Labelled "5W LS". The firmware treats the speaker as small and rolls off below 300 Hz.
- Microphones: one MEMS microphone (MK1, LMA2718) on the codec's MIC2R/LINE2R input, exposed by the ALSA driver as `Mic3R`. Wago connectors J2 and J9 accept an external microphone or line input.

No separate audio board. The speaker is a bare 2-wire driver.

### 7.2 Connection to the RK3566

- Control: I2C3 (`/dev/i2c-3`, hardware controller, 400 kHz) on header pins 3 and 5. Overlay `i2c3-pihat` re-muxes the RK3566's i2c3 from its M1 pins (where the vendor DTB uses it for the FUSB302 USB-C PD controller) to M0 and disables the FUSB302 node. USB-C PD negotiation is lost; the board still takes default 5 V over USB-C and is battery powered in the robot anyway.
- Audio data: I2S. Overlay `aic3104-i2c3` enables `i2s3_2ch`, defines the codec under i2c3, and builds a `simple-audio-card` named `aic3104` with the RK3566 as bit-clock and frame master. The card name matches the Pi build so `plughw:aic3104` works on both.
- Driver: `snd-soc-tlv320aic3x` is not built in the vendor kernel, so it ships as a DKMS module (`deploy/audio/aic3x-dkms/`). `aic3104-init.service` sets mixer levels at boot: PCM volume, Line DAC volume, Line Playback switch on (this is the LOP output mute; off silences the robot), Mic3R to the right PGA.
- Playback: `robotd` spawns `aplay` per sound on `plughw:aic3104`. Capture for the petting classifier is `plughw:aic3104,0`.

### 7.3 I2S pin note (verify on hardware, fix on the final board)

The public HAT wires BCLK to header pin 12, which on a Raspberry Pi is PCM_CLK. On the Radxa Zero 3W pin 12 is I2S3_MCLK_M0 and the bit clock, I2S3_SCLK_M0, is pin 11 (GPIO3_A3). LRCK (pin 35), SDI (pin 38) and SDO (pin 40) line up with the HAT's WCLK, codec DOUT and codec DIN. The microduck overlay refers to a sibling file `aic3104-pihat.dts` for "the I2S/MCLK/clocking rationale" that is not in the public repo. Before using rev C1 unmodified, check whether the shipped robot bridges pin 11 to the HAT's BCLK net or uses a variant. It is one wire on the prototype and one trace on the forked board.

Source: `microduck/deploy/audio/*.dts`, `aic3104-init.sh`, `aic3x-dkms/`, `microduck/robotd/src/sound.rs`, HAT schematic sheet 6/6, Radxa header table, RK3568 pinctrl.

## 8. Camera

Unchanged.

- Sensor: Sony IMX219, the Raspberry Pi Camera Module v2 class. The vendor kernel identifies it as `imx219 2-0010` (I2C bus 2, address 0x10, over the CSI connector's I2C pair). Pollen's store calls it a wide-angle camera and the ideas doc assumes about 62° horizontal FOV, so it may be a wide-lens IMX219 module rather than the stock Pi lens. The RL model places `head_camera` in the head next to the ToF.
- Physical: the Zero 3W's 22-pin, 0.5 mm pitch MIPI CSI connector. A Pi camera module has a 15-pin, 1.0 mm connector, so a 22-pin to 15-pin FPC adapter cable is required (Radxa sells one for the Zero 3W/3E). Four lanes on the connector; the IMX219 uses two.
- Device tree: overlay `radxa-zero3-rpi-camera-v2`. Armbian ships it without the `rk3568-` prefix, so `setup-board.sh` copies it to `rk3568-radxa-zero3-rpi-camera-v2.dtbo` before naming it in `overlays=`. Without the overlay there is no `/dev/video*` and nothing in dmesg, which looks exactly like an unplugged camera.
- Capture: Rockchip rkisp on the vendor kernel. The capture node is found by card name `rkisp_mainpath` (node numbers change between boots). The sensor boots in 3280 x 2464 and `mediad` pins it to 1920 x 1080 SRGGB10 with `media-ctl` at startup, then the ISP scales to 720p. Single-plane NV12 or UYVY, at least three capture buffers.
- Encode: `mpph264enc` through `/dev/mpp_service` (Rockchip MPP), not V4L2 M2M. `/dev/mpp_service` and `/dev/rga` need a udev rule for the `video` group, and `mediad.service` needs `SupplementaryGroups=video`.
- 3A: Rockchip's `rkaiq_3A_server` with Radxa's IMX219 IQ file does white balance, colour and noise reduction; its auto-exposure fires once, so `mediad` runs its own exposure loop.
- Mounting: the sensor is mounted rotated (the docs disagree on a quarter turn or upside down). Nothing rotates in the pipeline; the viewer rotates. `videoflip` was measured to throttle the SoC to 408 MHz.

Source: `microduck/docs/project/media-bringup.md`, `scripts/setup-board.sh` (`configure_camera`), `scripts/setup-rkaiq.sh`, `mediad/src/pipeline.rs`, `mediad/src/exposure.rs`.

## 9. Time-of-flight sensor

Same sensor and bus. On the prototype it is wired to the header directly instead of through the HAT's Qwiic port.

Prototype wiring: SDA to header pin 3, SCL to pin 5, 3V3 from pin 1, GND, and one 10 k pull-up from each of SDA and SCL to 3V3 near the SBC end. The HAT carried the only pull-ups on this bus, so without it the bus has none. The `i2c3-pihat` overlay is still required (it moves i2c3 to these pins and disables the FUSB302), and the udev rule that makes `/dev/i2c-pihat` follows the controller, not the HAT, so `tofd` needs no change. With the codec gone the ToF is the only device on the bus; if the head cable makes 400 kHz flaky, 200 kHz costs nothing. The sensor takes 3.3 V IO on the stock breakouts.

Reference and unchanged details:

- Sensor: ST VL53L8CX (the daemon also accepts a VL53L5CX and probes the device ID). 8 x 8 zones, 45° x 45° field of view, 15 Hz. In the head next to the camera (`tof` site in the RL model). A stock breakout with a Qwiic or STEMMA QT connector. Not every duck has one; `tofd` tolerates its absence.
- Bus on the shipped robot: hardware I2C3, shared with the audio codec, through the HAT's Qwiic J5 (JST SH 1.0 mm, 3V3) and a cable to the head. The HAT provides the only pull-ups, one 10 k pair (R12/R13). The overlay comment warns that head-cable capacitance can stretch rise times and suggests 200 kHz if 400 kHz proves flaky.
- Address: 0x29 default; 0x52 also tried because a prototype once moved it.
- Device path: `/dev/i2c-pihat`, a udev symlink following the i2c3 controller at `fe5c0000.i2c`, with `/dev/i2c-3` as fallback.
- Access: `tofd` runs unprivileged as user `tofd` in the `i2c` group and uses `I2C_RDWR` ioctls in 2 KiB chunks so the codec is never locked out for long.
- Bring-up cost: about 90 KB of firmware uploaded at every sensor start, a few seconds at 400 kHz. This is why the ToF is its own daemon.

On the final board the Qwiic port and pull-ups for the ToF come back on I2C3 exactly as the HAT has them.

Source: `microduck/tof/src/main.rs`, `sensor.rs`, `vendor/platform.c`, `systemd/tofd.service`, `deploy/audio/i2c3-pihat.dts`, `kinematics/src/tof.rs`.

## 10. Power: 6S LiPo

Decision: a 6S LiPo replaces the NP-F550.

Voltage fit.

| | Volts |
|---|---|
| 6S fully charged (4.20 V/cell) | 25.2 |
| 6S nominal (3.70 V/cell) | 22.2 |
| 6S empty under load (3.50 V/cell) | 21.0 |
| J288 range, recommended | 6.4 to 25.2, recommended 25.2 |
| HAT +BATT input (AP63205 buck) | 5 to 28 |

- The J288's recommended 25.2 V is exactly a full 6S, so the servos run at their design point and deliver the 35 rad/s rating rather than 16.5 rad/s at 12 V. Standard 4.20 V/cell packs only: an HV LiPo (4.35 V/cell, 26.1 V) exceeds the servo's maximum. The S288 is ruled out at 12.6 V.
- No headroom above full charge. The J288 fault table has transient (0x02) and sustained (0x04) over-voltage codes, and fifteen servos braking together can pump the rail above pack voltage momentarily. A LiPo absorbs regeneration well, but a pack straight off the charger has nowhere to put it. The bridge reads the fault flags; a TVS or a small bulk capacitor bank at the distribution board is cheap insurance; charging to 4.15 V/cell removes the issue at little cost if it appears.
- SBC and MCU supply on the prototype: a standalone buck converter from the pack to 5 V. Spec: input rated comfortably above 25.2 V (30 V or more, so a full pack plus regeneration transients is inside the rating), 5 V output at 3 A continuous (Radxa specifies a 5 V 2 A supply for the Zero 3W and the bridge adds under 0.2 A), synchronous, low ripple, and an enable pin or a switch so the robot has a power switch. Feed the header's 5 V pins 2 and 4 (with the Zero 3W's USB-C then unused for power) and the RP2350's 5 V input (VSYS on a Pico 2) from the same output. Keep the buck's ground return short to the SBC and away from the servo power path, and add bulk capacitance at its input: the 6S rail will carry the servos' switching noise.
- Battery detection (the HAT's IO_06) is gone with the HAT; the bridge's pack-voltage reading replaces it.
- The final board reinstates the HAT's own supply. Its power sheet is annotated for 24 V systems ("7s/25c, from 21V to 29V") and the AP63205 accepts 32 V, so that section of the HAT carries across without change.

Current and distribution.

- Unitree's no-load figure is 0.45 A at 25.2 V per servo, measured spinning unloaded, so it is not the idle draw. Standing and walking draw need measuring on the robot before pack capacity and fuse are fixed.
- Servo power does not go through the HAT or the bridge board. The HAT's servo-power link is marked 3 A. Use a separate distribution board (XT30 or XT60 in, fuse, per-limb branches out). The HAT and bridge see only +BATT for their own regulators and the data line.
- The servo pigtail is PH 2.0 (SIGNAL, VCC, GND), about 2 A per contact. Branch VCC and GND from the distribution board per servo or per limb and share only the signal wire per bus segment.
- The bridge measures pack voltage with its own divider. The J288 reports supply voltage in 0.5 V steps, only eight or nine steps across a 6S discharge, too coarse for a gauge or a shutdown decision.

Firmware constants that change.

- `BATTERY_FULL_V` and `BATTERY_EMPTY_V` in `duck-control/src/model.rs` are 8.2 and 6.6 for the 2S pack. For 6S they become about 25.2 and 21.0. The `battery_empty_shutdown` sit-and-power-off behaviour keys off the same floor and matters more with a bare LiPo than with the protected NP-F550.
- `nominal_voltage` in `robotd.toml` (7.4, used by the optional `voltage_adapt` scaling) becomes 22.2 if that feature is used.
- Volts come from the bridge's ADC in the state frame instead of being averaged from servo registers.

Pack safety and mass.

- The NP-F550 carried its own protection circuit. A bare LiPo does not, so the distribution board needs a fuse sized for the measured peak, and the firmware low-voltage shutdown is the only cell protection in the loop. A balance connector must stay reachable for charging.
- A 6S 1000 to 1300 mAh pack is roughly 150 to 200 g against about 100 g for the NP-F550, on top of the 255 g the J288s add over the XL330s. Both numbers go into the RL project's robot model.

Reference, shipped robot: removable Sony NP-F550 type 2S Li-ion, firmware maps 8.2 V to full and 6.6 V under load to empty and shuts down at 6.6 V, about one hour of run time, servo rail straight from the battery through the HAT, no fuel gauge or ADC (voltage read from the servos' own supply register).

Source: `microduck/duck-control/src/model.rs`, `deploy/robotd.toml`, HAT schematic sheet 3/6, Pollen store page, Unitree product page.

## 11. Carrier board plan

### 11.1 Prototype: no HAT

- Zero 3W with nothing on the header but wires: 5 V in on pins 2 and 4 from the buck converter, UART2 on pins 8 and 10 to the bridge, I2C3 on pins 3 and 5 to the ToF with added pull-ups, 3V3 from pin 1 for the ToF.
- Bridge: a Pico 2 (or similar) on the buck's 5 V, a small buffer board for the 6 Mbps servo line, and an LSM6DSV16X breakout on SPI mounted where the trunk IMU sits.
- Servo power from the distribution board; the bridge and SBC share the buck.
- No audio. `[audio] enabled = false`, and the audio steps of `setup-board.sh` skipped.
- What this buys: no HAT-specific unknowns (the I2S pin question, the 1 Mbps direction buffer) on the critical path to a walking robot, and every wire on the header is one you put there.

### 11.2 Final: one board derived from the HAT

Fork `elec_RPI_Robot_HAT` in KiCad 9 and make one board that replaces HAT, `imu_to_dxl`, bridge and buck.

Keep: the AP63205 buck and LM5050-1 ideal diode (5 to 28 V in), battery detection on IO_06, TLV320AIC3104 with its 12 MHz oscillator, PAM8406 amplifier and speaker Wago, MEMS microphone and external mic Wagos, Qwiic J5 on I2C3 for the ToF, the 40-pin footprint.

Delete: the Dynamixel TTL buffer and auto-direction network, the SIT3088E RS485 path and its connectors, the BMI088, the HAT EEPROM footprint.

Add: the RP2350 (bare chip or module), the LSM6DSV16X on SPI, 3.3 V for both, the 6 Mbps bus buffers (74LVC1G125/126 or equivalent, one pair per bus segment) with PIO-driven direction, PH 2.0 servo signal connectors per segment, a pack-voltage divider into an RP2350 ADC pin, and a USB or SWD header for the MCU. Route UART2 from header pins 8 and 10 straight to the RP2350.

Fix: BCLK onto header pin 11 (I2S3_SCLK_M0), section 7.3.

Servo power remains on the separate distribution board.

## 12. Calibration and risks

- IMU mount quaternion. Derived on the bench with the robot upright and tilted about known axes, then stored in `robotd.toml`. Wrong, it walks badly without a fault.
- Joint zero offsets and directions. The J288 reports a multi-turn rotor position relative to power-up plus a 13-bit absolute output-side encoder. Each joint needs a zero offset and a sign that map to the RL model's `DEFAULT_POSITION` frame. Store per-joint calibration on the bridge or in `robotd.toml`, and make the bench procedure repeatable.
- Stiffness. The XL330 P gain of 200 and its scaled variants are what the shipped policies were trained against through the BAM actuator model. J288 kp and kd are new parameters and the RL project will re-fit its actuator model to the J288; do not expect the shipped policies to transfer.
- Over-voltage at full charge (section 10) and the J288's 6.4 V floor are the two power edges; only the first one is near.
- 4 Mbps on UART2 under the vendor kernel is documented but unexercised. The fallback is 2 Mbps.
- The J288 signal logic level is unconfirmed. Buffer until it is.
- Supply noise. The SBC and IMU now sit on a buck fed from the servo pack rather than the HAT's filtered rail. Camera and IMU noise are the symptoms to watch; input bulk capacitance and grounding are the fixes.

## 13. Open items to verify on hardware

- The J288 SIGNAL logic level and idle state, with a scope.
- Standing and walking current of fifteen J288 on 6S, to size pack, fuse and distribution wiring, and whether over-voltage faults appear at full charge.
- That the vendor kernel clocks UART2 to 4 Mbps.
- How the shipped robot's HAT gets BCLK onto RK3566 I2S3_SCLK (pin 11 versus pin 12), and whether the shipped HAT is rev C1 or a variant. Needed for the final board, not the prototype.
- Which IMX219 module and lens is fitted, and the FPC adapter used.
- The IMU mount quaternion and the fifteen joint zero offsets, on the assembled robot.

## 14. References

- microduck firmware: https://github.com/pollen-robotics/microduck (local copy in `../microduck`)
- microduck RL: https://github.com/pollen-robotics/microduck_rl (local copy in `../microduck_rl`)
- RPI Robot HAT (KiCad, BOM, schematic PDF): https://github.com/pollen-robotics/elec_RPI_Robot_HAT
- rustypot (the shipped Dynamixel client, for reference): https://github.com/pollen-robotics/rustypot
- Radxa Zero 3 docs and header table: https://docs.radxa.com/en/zero/zero3 and https://docs.radxa.com/en/zero/zero3/hardware-design/hardware-interface
- Rockchip RK3568 datasheet (shared with RK3566), UART "up to 4Mbps": https://dl.radxa.com/rock3/docs/hw/datasheet/Rockchip-RK3568-Datasheet-V1.0-20201210.pdf
- RK3568 pinctrl (UART9_M1, I2S3_M0, I2C3_M0 pin mux): https://github.com/torvalds/linux/blob/master/arch/arm64/boot/dts/rockchip/rk3568-pinctrl.dtsi
- Radxa camera FPC 22-pin to 15-pin cable for Zero 3W/3E: https://evelta.com/radxa-camera-8m-imx219-for-raspberry-pi-rock-series/
- Pollen Microduck product page (1 GB + 32 GB, NP-F550): https://store.pollen-robotics.com/products/microduck
- Unitree digital servo specifications: https://www.unitree.com/DigitalServo/
- Unitree J288/S288 manual and STM32/Python notes: `docs/datasheets/` in this repo
- RP2350 datasheet (UART fractional divider, PIO, erratum E9): https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- ROBOTIS Dynamixel Shield reference circuit (the HAT's TTL buffer): https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/
