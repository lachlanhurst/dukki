# Microduck Unitree: audio setup

How to bring up the head speaker (MAX98357A) and microphone (INMP441) on the Waveshare CM4-NANO-A with a Radxa CM4 (RK3576) running Armbian. Follows on from `compute-setup.md`. The hardware reasoning is in `hardware.md` section 7.

Date: 24/09/2026. Written against vendor kernel 6.1.115-vendor-rk35xx, board device tree `rk3576-radxa-cm4-io.dtb`. Installed the same day with the amplifier wired: the card registers, playback runs at 48 kHz stereo S32_LE through dmix and is heard clearly on the speaker. With the microphone wired, a 440 Hz tone from the speaker shows up in the capture about 700 times above the neighbouring bins.

## 1. The short version

1. Wire the amplifier with the board powered off:

| MAX98357A | NANO-A header pin | Signal |
|---|---|---|
| VIN | 2 or 4 | 5 V |
| GND | 39, or any ground | ground |
| BCLK | 12 | SAI2_SCLK_M0 |
| LRC | 35 | SAI2_LRCK_M0 |
| DIN | 40 | SAI2_SDO_M0 |
| GAIN, SD | unconnected | 9 dB, (L + R) / 2 mono |

The speaker goes across the amplifier's + and − terminals only. The output is bridge-tied, so neither lead goes to ground. The INMP441, when fitted, shares pins 12 and 35, puts SD on pin 38, takes 3.3 V from pin 1 or 17, and has L/R tied to ground.

2. From the Mac:

```
scp -r scripts/compute/setup-audio.sh scripts/compute/audio duck@microduck.local:
ssh -t duck@microduck.local sudo bash setup-audio.sh
ssh -t duck@microduck.local sudo reboot
```

3. After the reboot:

```
ssh duck@microduck.local bash setup-audio.sh --check
ssh duck@microduck.local bash setup-audio.sh --test
```

`--check` should list a `microduck` card, with `simple-card` as its driver. `--test` sets the volume to 80 %, plays a 440 Hz tone and then a spoken "front centre".

## 2. What is installed

- `audio/microduck-nano-a-sai2.dts`, an Armbian user overlay. It enables `sai2`, adds a `rockchip,dummy-codec` node, and adds a `simple-audio-card` named `microduck` with the SAI as bit and frame master. The format is I2S with 2 slots of 32 bits, so BCLK is 64 × fs, which the INMP441 needs. The base tree's `sai2` node already selects the M0 pins without MCLK, so the overlay does not touch pinctrl. The SAI driver, dummy codec and simple card are all built into the vendor kernel.
- `audio/asound.conf`, installed as `/etc/asound.conf`. It makes the default ALSA device the new card: plug, then softvol (`Speaker`), then dmix for playback, and plug, then dsnoop for capture. dmix and dsnoop hold the hardware at 48 kHz, stereo, S32_LE, and let several processes play or record at once. Any existing `/etc/asound.conf` is saved as `.bak`.

Installation works as for the camera and head I2C: the script test-applies the overlay with `fdtoverlay` before installing, and `DRY_RUN=1` only compiles. See `camera-setup.md` section 3.

Volume: `amixer sset Speaker 60%`. The control appears only after the first playback, a softvol property, which is why `--test` plays a second of silence first. The GAIN pin on the breakout sets the amplifier's fixed gain if the range is wrong overall.

## 3. Notes

- The card is ALSA card 0 and the default device. The base tree's ES8388 card (IO-board codec, not fitted on the NANO-A) still registers as card 1 and logs `es8328 3-0010 ... -6` errors at boot. They are harmless.
- Microphone symptoms. With the INMP441 unpowered, SD floats: most left-slot words read exactly 0x00000000 and the rest a near-constant 0x78xxxx00 or 0x7Fxxxx00 pattern picked up from the clock edges. A working mic never gives an exact zero word, even in a silent room, because its noise floor sits around −87 dBFS. The right slot is always zero with L/R grounded.
- The mic needs a fraction of a second after the stream opens, while the bit clock starts and its filter settles, so discard the start of a fresh capture.
- The SAI exposes a set of `SAI2 ...` mixer controls (loopback, mono, path select). Leave them at their defaults.
- Firmware still to change: `robotd` passes `audio.device` to `aplay -D`, and the petting worker records from `<device>,0`, default `plughw:aic3104,0`. Point playback at `default` (or `speaker`) and capture at `mic`, so both go through the softvol and dsnoop defined here rather than taking the raw `hw` device.
