# Microduck Unitree: servo setup

How to take fifteen Unitree J288 servos out of the box and make them ready for the robot: assign bus IDs, record firmware versions, run a bench motion check, and label each unit for its joint. This is a bench procedure done once per servo before assembly. Runtime control from the bridge MCU is covered in `hardware.md` section 5.

Date: 07/09/2026.

## 1. Sources

- `docs/datasheets/无刷数字舵机J288S288使用手册.pdf`: the J288/S288 user manual. Wiring, protocol, fault codes.
- `docs/datasheets/unitree_servo_software.pdf`: the manual for Unitree's Windows GUI (电机调试助手, now shipped as "Unitree Motor Assistant"). Describes the tool, gives no download link.
- Unitree Motor Assistant repository: https://github.com/unitreerobotics/Unitree-Motor-Assistant. README lists J288/S288 and IM6014 as supported and Windows as the only platform.
- Unitree support centre, IM6014 Motor Development Guide, sections "Motor Debugging Assistant" and "Motor Configuration": https://support.unitree.com/home/en/Motor_IM6014%20Development%20Guide/Motor_debugging_assistant. This is the current user guide the README points at. It is written for the IM6014 and some of its features do not exist on the J288 (section 8).
- Unitree digital servo examples: https://github.com/unitreerobotics/digital_servo. Python and STM32 demos plus `specs/protocol.md`.
- Shipped robot ID order: `microduck/duck-control/src/model.rs` and `duck-ipc-proto/src/lib.rs`.

## 2. The one fact that shapes this procedure

The J288 protocol Unitree publishes is a 20-byte command and a 26-byte reply. There is no command in it to query, change or scan IDs, read the firmware version, change baud rate or enter the bootloader. Those functions exist only in the Windows GUI, which sends undocumented frames. The consequences:

- IDs must be set from a Windows PC with the GUI, one servo at a time, before the servo goes into the robot.
- The bridge MCU cannot assign IDs on the assembled robot. It can only find which IDs answer.
- A servo with an unknown ID can still be found: the GUI's Query ID scans the bus and lists every servo present.

If we ever want the bridge to do this itself, capture the bus with a logic analyser while the GUI performs Query ID and Modify ID, and add those frames to the bridge firmware. They will be short, CRC32-protected frames using the same polynomial as the control frame. Until then, plan on the GUI.

## 3. What you need

Hardware:

- One J288 at a time on the bench bus. Fifteen in total.
- The PH 2.0 3-pin pigtail supplied with each servo (SIGNAL, VCC, GND).
- A USB to single-bus serial adapter that runs 6 Mbps half duplex. Unitree sells one as the "Single-Bus Serial-to-USB Module" variant on the digital servo shop listing. An FTDI FT232H board is the alternative. FT232R, CH340 and CP2102 adapters top out well below 6 Mbps and will not work. Unitree states the GUI has only been verified with FTDI parts.
- A bench supply, 12 V is enough for configuration and light motion tests, 25.2 V for a representative motion test. The manual's bench setup powers the servo through an XT30 lead. The servo draws under 0.5 A unloaded.
- A Windows 10 or 11 PC. The GUI does not run on macOS or Linux.
- Labels for the servos.

Software:

- Unitree Motor Assistant. Clone or download the repository as a zip. There are no GitHub releases: the exe, `Unitree Motor Assistant.exe`, and its Qt 6.6.3 DLLs are committed to the repo root. Keep the folder intact and run the exe from it.
- FTDI VCP driver. Unitree links a zip on the IM6014 download page. The standard FTDI installer from ftdichip.com is the same thing.
- Optional: the `digital_servo` repository for the Python demo, useful as a second opinion from a Mac or Linux machine once IDs are set.

## 4. ID plan

The J288 ID field is 4 bits. Valid servo IDs are 0 to 14 and 15 is broadcast with no reply. Fifteen joints fit exactly. Give each servo the index of its joint in the firmware's `JOINT_NAMES` array, so the ID doubles as the array index and no translation table is needed on the bridge.

| ID | Joint | Group | Shipped Dynamixel ID (reference) |
|---|---|---|---|
| 0 | left_hip_yaw | left leg | 20 |
| 1 | left_hip_roll | left leg | 21 |
| 2 | left_hip_pitch | left leg | 22 |
| 3 | left_knee | left leg | 23 |
| 4 | left_ankle | left leg | 24 |
| 5 | neck_pitch | neck and head | 30 |
| 6 | head_pitch | neck and head | 31 |
| 7 | head_yaw | neck and head | 32 |
| 8 | head_roll | neck and head | 33 |
| 9 | mouth | neck and head | 34 |
| 10 | right_hip_yaw | right leg | 10 |
| 11 | right_hip_roll | right leg | 11 |
| 12 | right_hip_pitch | right leg | 12 |
| 13 | right_knee | right leg | 13 |
| 14 | right_ankle | right leg | 14 |

After changing IDs, power-cycle the servos before testing them on the bridge: on the bench, two servos with new IDs, plugged into a live harness, stayed silent until the battery was disconnected and reconnected.

Two IDs on one bus break communication for every servo on it. The bus is run as three segments from the bridge, one per leg and one for the neck and head (`hardware.md` 5.1 and 5.6), so keep the IDs globally unique across all three. It costs nothing and keeps a mis-plugged limb from silently colliding.

Spares get ID 14 and a label saying so, then are re-addressed when fitted.

## 5. Bench wiring

From figure 2 of the J288 manual.

```text
   PC  --- USB-C ---  Unitree single-bus to USB module  --- PH 2.0 3-pin ---  J288
                                                     |
                                       XT30 from bench supply (12 to 25.2 V)
```

- SIGNAL, VCC and GND on the PH 2.0 connector. Power reaches the servo through the module's XT30 input in this setup. On the robot, servo power comes from the distribution board instead (`hardware.md` section 10).
- The green LED blinks when the servo has power and has booted.
- If using an FT232H board instead of Unitree's module, wire TX and RX together onto SIGNAL through a series resistor, or use the FTDI's single-wire mode, and mind the logic level: the J288 bus is 5 V, and each servo pulls the line up weakly towards 5 V (`hardware.md` section 5.6).

## 6. First connection

1. Install the FTDI driver and plug in the module. Note the COM port in Device Manager. Unplug and replug if more than one is listed.
2. Lower the COM port latency. Device Manager, the port's Properties, Port Settings, Advanced, set Latency Timer to 1 ms. Without this Windows caps the tool's polling at about 60 Hz. Pages 6 to 8 of `unitree_servo_software.pdf` show the dialogs.
3. Start `Unitree Motor Assistant.exe`. Switch the interface to English if preferred (function area, top of the window).
4. Function area: select motor model J288, select the COM port, connect.
5. Click Query ID. With one servo on the bus the tool lists its ID and switches the control ID to it. Unitree's other motors ship at ID 0. Record what the J288 actually reports.
6. The receive area should show live voltage, temperature, position and a zero error code. If nothing updates, check power, the COM port, the latency setting and the cable before anything else.

## 7. Per-servo procedure

Do this with exactly one servo connected. Repeat fifteen times.

1. Connect, power on, Query ID. Confirm one servo answers.
2. Enter the target ID from the table in section 4 and click Modify ID. The tool reports success and moves its own control ID to the new value.
3. Power cycle the servo. Query ID again and confirm the new ID persisted.
4. Click Query Version. Record the firmware version.
5. Bench motion check (section 9). Confirm the servo moves, the reported position tracks, and no error code appears.
6. Label the servo with its ID and joint name. Put the label where it stays visible after assembly.
7. Fill in the row in section 11.

Do not click Update Firmware, Temporarily Disable Boot or Restore Motor Mode during setup. They are recovery and update tools, not part of configuration. Section 8 says what they do.

## 8. What the GUI's buttons do on a J288

From `unitree_servo_software.pdf` and the IM6014 guide. Where the two disagree, the J288 PDF wins.

| Button | Effect | Use on J288 |
|---|---|---|
| Query ID | Scans the bus, lists every ID present. Hover over the button to see the list if there are several | Yes. Also the way to find a lost ID |
| Modify ID | Writes the target ID to the connected servo | Yes, one servo on the bus |
| Query Version | Reads firmware version | Yes, record it |
| Select Firmware, Update Firmware | Loads a .bin and writes it to the servo | Only with a file from Unitree. Do not power off during the write |
| Enable Auto Boot, Disable Auto Boot | Controls whether the motor boots into the application or stays in the bootloader | Not available on J288/S288 (stated in the PDF) |
| Temporarily Disable Boot, Exit Temporary Boot | Puts the servo into the bootloader until power cycle | Recovery only |
| Restore Motor Mode | Returns from bootloader to normal operation | Recovery only |
| Modify Baud Rate | Toggles 6 Mbps and 4 Mbps | IM6014 feature. Not in the J288 PDF and the J288 manual says the baud is fixed. Do not try it on a J288 until Unitree confirms |
| Power-On Position Mode, Clear Error, Motor Reset | IM6014 features | Not in the J288 PDF. Treat as unavailable |

## 9. Bench motion check

All values in the tool are output-side units after gear ratio 288.35. Servo mode 0 is stop with the output locked, mode 1 is the hybrid closed loop:

tau = tau_ff + kp (p_des - p) + kd (w_des - w)

Speed check, matching Unitree's own example:

1. Send area: mode 1, timeout off, torque 0, speed 0.5 rad/s, position 0, Kp 0, Kd 1.0.
2. The output turns slowly at about 0.5 rad/s. The receive area speed reads about 0.5. Set mode 0 to stop.

Position check:

1. Mode 1, timeout off, torque 0, speed 0, Kd 0.05, Kp 0.5, then set position to 0.5 rad and back to 0.
2. Raise Kp in small steps if the response is soft. Large Kp or Kd values cause hard oscillation. Start small every time.

Notes:

- With timeout on, the servo executes one command and expects the next within about a second, so the tool must be streaming. With timeout off, the last command holds until replaced. Timeout off is convenient on the bench. The bridge will run with timeout on so a dead link stops the joints.
- Tick the position and speed boxes to plot them and confirm the response looks clean.
- The rotor position is multi-turn and relative to power-up. The 13-bit output encoder gives absolute position within one turn. Joint zero offsets are a separate calibration on the assembled robot (`hardware.md` section 12).

## 10. Faults

Error codes appear in the receive area as a bit mask. From table 1 of the J288 manual:

| Bit | Code | Meaning |
|---|---|---|
| 0 | 0x01 | over-current |
| 1 | 0x02 | transient over-voltage |
| 2 | 0x04 | sustained over-voltage |
| 3 | 0x08 | under-voltage |
| 4 | 0x10 | chip over-temperature |
| 5 | 0x20 | MOSFET over or under temperature |
| 6 | 0x40 | MOSFET temperature sensor fault |
| 7 | 0x80 | case over or under temperature |
| 8 | 0x100 | case temperature sensor fault |
| 9 | 0x200 | winding over-temperature |
| 10 | 0x400 | rotor encoder 1 fault |
| 11 | 0x800 | rotor encoder 2 fault |
| 12 | 0x1000 | output encoder fault |
| 13 | 0x2000 | stored data fault |
| 14 | 0x4000 | abnormal reset |
| 15 | 0x8000 | waiting for unlock |
| 16 | 0x10000 | driver verification fault |
| 17 | 0x20000 | calibration mode |
| 18 | 0x40000 | communication checksum fault |
| 19 | 0x80000 | driver version too old |
| 20 | 0x100000 | firmware model mismatch |
| 21 | 0x200000 | encoder angle abnormal |

Warning bit 0 (0x01) is a low-voltage supply warning. The J288 PDF has no Clear Error button. Fix the cause and power cycle. A servo that repeats an encoder or stored data fault after power cycling goes back to Unitree, not into the robot.

Under-voltage on the bench usually means the supply current limit is too low or the XT30 lead is marginal. Over-voltage at 25.2 V with rapid decelerations is the regeneration case discussed in `hardware.md` section 10.

## 11. Record

Fill in as each servo is done. Keep this table current, it is the only place the servo to joint mapping and firmware set live until the bridge reads them at boot.

| ID | Joint | Servo serial or batch | Firmware | Date set | Notes |
|---|---|---|---|---|---|
| 0 | left_hip_yaw | | | | |
| 1 | left_hip_roll | | | | |
| 2 | left_hip_pitch | | | | |
| 3 | left_knee | | | | |
| 4 | left_ankle | | | | |
| 5 | neck_pitch | | | | |
| 6 | head_pitch | | | | |
| 7 | head_yaw | | | | |
| 8 | head_roll | | | | |
| 9 | mouth | | | | |
| 10 | right_hip_yaw | | | | |
| 11 | right_hip_roll | | | | |
| 12 | right_hip_pitch | | | | |
| 13 | right_knee | | | | |
| 14 | right_ankle | | | | |
| spare | | | | | |

## 12. After assembly

- Put all fifteen on the bridge bus, power up, and have the bridge poll IDs 0 to 14 once. Every ID must answer and none may answer twice. A missing ID is a cable or a mislabel. Two replies to one ID is a duplicate and must be fixed before anything moves.
- Compare the firmware versions from section 11. Mixed versions are acceptable until Unitree documents a difference, but note them.
- Move on to joint zero offsets and directions (`hardware.md` section 12).

## 13. Open items

- The J288 factory default ID. Confirm on the first unit and note it here.
- Whether the GUI's J288 profile exposes a baud change. The J288 manual says 6 Mbps fixed. If 4 Mbps is available it would let the compute module's own UART talk to the servos for bench work, though not for the robot (`hardware.md` 5.2).
- Whether Unitree publishes J288 firmware files. None are listed on the digital servo download page at the time of writing.
- Frame formats for Query ID and Modify ID, if we decide the bridge should be able to re-address a servo in place.

## 14. References

- Unitree Motor Assistant (Windows exe, BSD-3): https://github.com/unitreerobotics/Unitree-Motor-Assistant
- Unitree digital servo examples and protocol spec: https://github.com/unitreerobotics/digital_servo
- Digital servo download page (manuals and examples, no GUI link): https://www.unitree.com/download/DigitalServo
- IM6014 guide, Motor Debugging Assistant: https://support.unitree.com/home/en/Motor_IM6014%20Development%20Guide/Motor_debugging_assistant
- IM6014 guide, Motor Configuration (ID change walkthrough): https://support.unitree.com/home/en/Motor_IM6014%20Development%20Guide/Motor_configuration
- GO-M8010-6 guide, Motor Debugging Assistant (older write-up of the same tool): https://support.unitree.com/home/en/Motor_SDK_Dev_Guide/Motor_debugging_assistant
- Brushless digital servo and single-bus USB module: https://shop.unitree.com/products/brushless-digital-servo
- FTDI driver zip linked by Unitree: https://oss-global-cdn.unitree.com/static/a14bc99cfd6842b78120150494fa639f.zip
