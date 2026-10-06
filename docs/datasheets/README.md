# Datasheets and schematics

The docs refer to vendor documents in this folder. They belong to their publishers and are not redistributed here: the folder is gitignored apart from this list. Download each one from its source into this folder under the file name shown, so the paths in the docs resolve.

| File | What it is | Source |
|---|---|---|
| `无刷数字舵机J288S288使用手册.pdf` | Unitree J288/S288 user manual: wiring, protocol, fault codes | Unitree digital servo download page, https://www.unitree.com/download/DigitalServo |
| `unitree_servo_software.pdf` | Manual for Unitree's Windows motor tool (Unitree Motor Assistant) | Unitree digital servo download page, https://www.unitree.com/download/DigitalServo |
| `STM32-HAL通讯例程说明.docx` | Unitree's notes on the STM32 HAL servo example | Unitree; source not recorded. The examples themselves are at https://github.com/unitreerobotics/digital_servo |
| `python通信例程说明.docx` | Unitree's notes on the Python servo example | Unitree; source not recorded. The examples themselves are at https://github.com/unitreerobotics/digital_servo |
| `CM4-NANO-A_SchDoc.pdf` | Waveshare CM4-NANO-A carrier schematic | Waveshare wiki, https://www.waveshare.com/wiki/CM4-NANO-A (Resources) |
| `WeAct-STM32G474CoreBoard_V10_SchDoc.pdf` | WeAct STM32G474 core board schematic, QFN48 variant, v1.0 | https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard |
| `WeAct-STM32G474CoreBoard_V10 Board Shape 外形.pdf` | WeAct STM32G474 core board outline drawing | https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard |
| `lsm6dsv16xtr.png` | Pinout image of the generic LSM6DSV16X breakout used as the trunk IMU | The module's seller listing; source not recorded |

Related datasheets the docs cite by link only: the STM32G474 datasheet DS12288 and reference manual RM0440, the LSM6DSV16X and BMI088 datasheets, and the Radxa CM4 schematic. Their URLs are in `docs/hardware.md` section 14.
