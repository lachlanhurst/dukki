# Microduck Unitree: camera setup

How to bring up the IMX219 on the Waveshare CM4-NANO-A with a Radxa CM4 (RK3576) running Armbian, and what was learnt on 22/09/2026 working it out. Follows on from `compute-setup.md`. The hardware reasoning is in `hardware.md` section 8.

Date: 22/09/2026. Written against Armbian 26.8.2, vendor kernel 6.1.115-vendor-rk35xx, board device tree `rk3576-radxa-cm4-io.dtb`. Installed and verified the same day: the sensor probes and the ISP main path streams 720p NV12. The 3A engine and an IMX219 tuning file for this ISP followed the same evening (section 6); with them the ISP output is auto exposed and colour corrected.

## 1. The short version

1. Plug the camera into the NANO-A's 15-pin CSI connector, contacts facing the board as marked, board powered off.
2. From the Mac:

```
scp -r scripts/compute/setup-camera.sh scripts/compute/camera duck@microduck.local:
ssh -t duck@microduck.local sudo bash setup-camera.sh
ssh -t duck@microduck.local sudo reboot
```

`-t` gives `sudo` a terminal for its password prompt; without it the command appears to hang. If `microduck.local` itself hangs, use the IP and see the avahi note in `compute-setup.md` section 5.5.

3. After the reboot:

```
ssh duck@microduck.local bash setup-camera.sh --check
```

Expect an `imx219` probe line in dmesg, `/dev/media0`, and a set of `/dev/video*` nodes named for rkisp and rkcif. No nodes and no dmesg line means the sensor did not answer on I2C: check the cable seating and orientation first, the overlay second.

4. Install the 3A engine and tuning file (section 6), as root:

```
scp -r scripts/compute/setup-camera-3a.sh scripts/compute/camera root@microduck.local:
ssh root@microduck.local bash setup-camera-3a.sh
ssh root@microduck.local bash setup-camera-3a.sh --check
```

Then `scripts/compute/camera/snap.sh --isp` from the Mac should give a normally exposed, neutral picture.

## 2. Why this shape

The camera needs a device-tree overlay. Without it the vendor kernel creates no video nodes and logs nothing, which is indistinguishable from an unplugged camera. The Armbian image boots Radxa's CM4 IO board tree, which enables no CSI receiver, and Armbian ships no camera overlay for the Radxa CM4 at all. The RK3576 camera overlays it does ship are for the ArmSoM CM5 on a Pi CM4 IO board (different I2C and CSI receiver assignment) and the reComputer RK3576 devkit, so neither can be used as is.

Radxa's own overlay package (`radxa-pkg/radxa-overlays`) has the right one: `radxa-cm4-rpi-cm4-io-radxa-camera-8m-219-cam0.dts`, the Radxa CM4 on a Raspberry Pi CM4 IO board with an IMX219 on CAM0. The NANO-A schematic shows its single CSI connector is wired exactly as Pi CAM0:

| NANO-A CSI connector | Net on the schematic | CM4 pin | RK3576 side | In the overlay |
|---|---|---|---|---|
| 1 to 9, data and clock pairs | CAM0_D0, CAM0_D1, CAM0_CLK | CAM0 lane pins | CSI1, two lanes | `csi2_dphy1`, `mipi1_csi2`, `rkcif_mipi_lvds1`, `rkisp_vir1` |
| 11, PWDN | CAM_GPIO | 97 | GPIO2_C5 | fixed regulator, enable active high |
| 12, MCLK | not connected | | | module's own 24 MHz oscillator |
| 13, 14, SCL and SDA | ID_SCL, ID_SDA, 4.7 kΩ pull-ups R20 and R21 | ID_SC, ID_SD | I2C6 on M3 pins | `&i2c6`, `i2c6m3_xfer` |

Two things follow. The camera I2C is I2C6, shared with header pins 27 and 28, not I2C0 as `hardware.md` said before this date. And Radxa's cam0 overlay applies unchanged. Radxa's cam1 overlay is the CAM1 position: CSI3 and I2C0 M1, the SDA0/SCL0 pair. It is the wrong one for this carrier.

`scripts/compute/camera/microduck-nano-a-imx219.dts` is that overlay with a header comment, its metadata block reworded, and `rockchip,camera-module-name` set to `rpi-camera-v2` so the tuning file name matches what the upstream `setup-rkaiq.sh` expects. Every label it references (`csi2_dphy0_hw`, `csi2_dphy1_hw`, `csi2_dphy1`, `mipi1_csi2`, `i2c6`, `i2c6m3_xfer`, `rkcif`, `rkcif_mmu`, `rkcif_mipi_lvds1`, `rkcif_mipi_lvds1_sditf`, `rkisp`, `rkisp_mmu`, `rkisp_vir1`, `gpio2`, `pcfg_pull_up`) exists in the `__symbols__` of the booted `rk3576-radxa-cm4-io.dtb`, and `fdtoverlay` merges it without error. The kernel has `VIDEO_IMX219`, `VIDEO_ROCKCHIP_CIF`, `VIDEO_ROCKCHIP_ISP`, `PHY_ROCKCHIP_CSI2_DPHY` and `ROCKCHIP_MPP_SERVICE` built in, not as modules, so nothing needs loading.

## 3. How it is installed

Armbian's `boot.cmd` applies two lists of overlays: `overlays=` from the kernel package's directory, with `overlay_prefix` (here `rk35xx`) prepended to each name, and `user_overlays=` from `/boot/overlay-user/`, with no prefix. The script uses the second. The compiled file lives outside the kernel package, so a kernel update leaves it in place, and there is no prefix trap of the kind the Zero 3W build had to work around. If an overlay fails to apply at boot, `boot.cmd` reloads the original tree, so a bad overlay costs the camera and not the boot.

`setup-camera.sh`:

1. Refuses to run on anything but the vendor kernel and checks the kernel headers, `cpp`, `dtc` and `fdtoverlay` are present. All are on the image as flashed.
2. Preprocesses the `.dts` with the kernel's `dt-bindings` headers, compiles it with `dtc -@`, and test-applies it to the tree named by `fdtfile` in `/boot/armbianEnv.txt`. An unresolved label fails here, loudly.
3. Installs the `.dtbo` into `/boot/overlay-user/` and adds the name to `user_overlays=`.
4. Does not reboot.

`DRY_RUN=1` does step 2 only, as any user. `sudo` needs a terminal for its password prompt, hence `ssh -t`. `--check` prints the state after a reboot.

## 4. Testing capture

Observed 22/09/2026, first boot with the overlay and a camera connected: `imx219 6-0010` probed, the D-PHY matched it, and both async notifiers completed. The sensor came up in its 3280x2464 mode.

Two media devices appear, and node numbers change between boots, so identify by name:

```
v4l2-ctl --list-devices
media-ctl -d /dev/media0 -p | grep '^- entity'     # rkcif: the sensor, D-PHY, CSI-2 receiver, stream_cif_mipi_id*
media-ctl -d /dev/media1 -p | grep '^- entity'     # rkisp: rkisp_mainpath, rkisp_selfpath, statistics, params
```

The sensor entity is `m00_f_imx219 6-0010`: module index 0, facing front (the overlay says so), I2C bus 6, address 0x10. The upstream Zero 3W build saw `m00_b_imx219 2-0010`. `mediad` matches the substring `imx219`, so neither change affects it. On this boot rkcif took `/dev/video0` to `10` and the ISP main path was `/dev/video11`, so "video0" is not the capture node here.

Everything below runs as `duck` without sudo, because the nodes are group `video` and the user is in it:

```
media-ctl -d /dev/media0 --set-v4l2 '"m00_f_imx219 6-0010":0[fmt:SRGGB10_1X10/1920x1080]'
MP=$(media-ctl -d /dev/media1 -p | awk '/entity .*rkisp_mainpath/{f=1} f&&/device node name/{print $4; exit}')
v4l2-ctl -d $MP --set-fmt-video=width=1280,height=720,pixelformat=NV12 --stream-mmap=4 --stream-count=30 --stream-to=/tmp/cam.nv12
```

This produced 30 frames of 1280x720 NV12 (41,472,000 bytes) on the first attempt. One frame is 1,382,400 bytes; cut one out with `dd bs=1382400 skip=29 count=1` and view it on the Mac with `ffmpeg -f rawvideo -pixel_format nv12 -video_size 1280x720 -i last.nv12 last.png`. Without 3A the picture is at a fixed exposure and nothing corrects colour or noise. That is correct at this stage.

The first frames were nearly black (luma 15 to 30 of 255, indoors in the evening) because nothing sets exposure. The sensor subdev is the node `media-ctl -d /dev/media0 -p` lists under the `imx219` entity, `/dev/v4l-subdev2` on this boot, and its controls on the vendor driver are:

| Control | Min | Max | Default |
|---|---|---|---|
| `exposure` | 0 | 4095 | 1575 |
| `analogue_gain` | 256 | 2816 | 512 |
| `gain` (digital) | 256 | 43663 | 256 |

```
SD=$(media-ctl -d /dev/media0 -p | awk '/entity .*imx219/{f=1} f&&/device node name/{print $4; exit}')
v4l2-ctl -d $SD --set-ctrl=exposure=4000
v4l2-ctl -d $SD --set-ctrl=analogue_gain=2816
```

With those the same scene captured at mean luma 99 with the full range in use.

Full resolution also works: pin the sensor to `SRGGB10_1X10/3280x2464` and set the main path to 3280x2464 NV12. A frame is 12,122,880 bytes and the stream runs at about 21 fps. In the same evening indoor light, with exposure at its maximum of 4095 and analogue gain at 2816, the full resolution frame came out at mean luma about 28, roughly four times darker than 1080p at the same settings. The 1080p mode is evidently gathering more light per output pixel. Full resolution needs daylight or software gain. The full resolution frame from 22/09/2026, as captured and with a 4x software luma boost, is in `docs/images/` as `camera-fullres-3280x2464-2026-09-22.jpg` and `camera-fullres-3280x2464-brightened-x4-2026-09-22.jpg`. A small C converter for NV12 to PNG, since neither the board nor the Mac has ffmpeg or numpy, took two seconds per 8-megapixel frame; pure Python takes minutes. Both frames, scaled to half size, are in `docs/images/` as `camera-first-frame-default-exposure-2026-09-22.png` and `camera-first-frame-2026-09-22.png`. The green cast is the ISP with no colour correction, and is what 3A tuning fixes. `v4l2-ctl` prints `VIDIOC_SUBDEV_S_CLIENT_CAP: failed: Inappropriate ioctl for device` on every subdev call on this kernel; it is harmless and the control still applies. A value outside the range is rejected silently, so read the control back. The digital `gain` control also works: at exposure 1200 and analogue gain 2816, gain 256, 1024 and 4096 gave mean luma 18, 27 and 61, compressed by the ISP gamma. Controls persist across format changes and captures, so a value left from an earlier test silently applies to the next one. These are the controls the upstream `mediad::exposure` loop drives, so that loop carries over with the ranges above.

### As the robot sees it

`mediad` pins the sensor to 1920x1080, asks the ISP main path for 1280x720 UYVY (single plane; two-plane NV12 stalls `v4l2src` at 20 fps), and meters mean luma towards 90 with exposure capped at 1200 lines (about 23 ms), then analogue gain to 11x, then digital gain to 16x. Reproducing that loop by hand on 22/09/2026, in the evening indoors, it ran out of range: exposure 1200, analogue 11x, digital 16x reached mean luma about 61 against the target of 90. The frame at that state is `docs/images/camera-robot-view-720p-2026-09-22.png`. It shows a markedly narrower field of view than the full resolution frame; see section 4a. The room is simply too dark for the daemon's exposure ceiling; the ceiling exists to keep motion blur down while walking.

### One command for a picture

`scripts/compute/camera/snap.sh`, run on the Mac, does the whole dance: finds the nodes by name, puts the sensor in its full frame mode, streams, writes exposure and gain once frames are flowing, pulls the last frame back, and writes `scratch/snap-<timestamp>-720p.png` (full width, 16:9) plus a full resolution PNG. By default it takes the raw sensor frame past the ISP and demosaics it with ffmpeg, which is the true picture; `--isp` takes what the robot's pipeline sees off the ISP main path instead. `EXPOSURE` (lines, default 4095), `GAIN` (total gain times 256, default 1536, 6x) and `HOST` are environment overrides. It needs ffmpeg and clang on the Mac; the unpacker for Rockchip's packed 10-bit layout is `unpack10.c` beside it and is compiled into `scratch/.bin` on first use.

## 4a. Field of view: the 1080p mode is a 39° crop

The vendor IMX219 driver in this kernel (`drivers/media/i2c/imx219.c`, Armbian `rk-6.1-rkr5.1`) has exactly two modes: 1920x1080 at 30 fps and 3280x2464 at 21 fps. Its 1080p register table sets the analogue crop window to x 680 to 2599 and y 692 to 1771 with binning off, so 1080p is a native-pixel centre crop of the array, 58.5 % of its width. With the module's 62° lens that is a horizontal field of view of about 39° and a vertical one of about 30°. Asking the driver for the sensor's binned 1640x1232 mode snaps to 1920x1080; the mode is not in the table.

This contradicts the upstream `mediad/src/camera.rs`, which states that on the Zero 3W the 1080p mode was a scaled full-frame readout and derives its nominal intrinsics from 62° (`fx` about 1062 at 1280 wide). On this board and kernel that geometry is wrong by a factor of about 1.7; a 39° crop at 1280 wide has `fx` about 1810. Either the pipeline is changed to deliver the full frame, or the intrinsics are corrected for the crop. Measured 22/09/2026 by comparing the 720p frame from the 1080p mode with the full resolution frame of the same scene: `camera-robot-view-720p-2026-09-22.png` against `camera-fullres-3280x2464-brightened-x4-2026-09-22.jpg`.

### Getting the full frame at 720p

Three routes, verified as far as stated:

1. Full sensor mode plus an ISP crop, verified working. Leave the sensor in 3280x2464 at 21 fps and have the ISP main path crop to 16:9 before it scales, otherwise it squashes the 4:3 frame into 16:9 anamorphically. The crop is a V4L2 selection on the main path node and it survives a later format set:

```
media-ctl -d /dev/media0 --set-v4l2 '"m00_f_imx219 6-0010":0[fmt:SRGGB10_1X10/3280x2464]'
v4l2-ctl -d $MP --set-selection=target=crop,top=310,left=0,width=3280,height=1845
v4l2-ctl -d $MP --set-fmt-video=width=1280,height=720,pixelformat=UYVY
```

   The driver rounds the height to 1846. Streams at 21.2 fps. CPU cost is nil: measured 22/09/2026 streaming 720p UYVY for about 25 s with nothing written, the whole system sat at 0.5 % busy across the eight cores from the 1080p mode and 0.4 % from the full sensor, against a 0.1 % idle baseline. The crop, debayer and scale all happen in the ISP; the extra work is DDR traffic for the 8-megapixel raw frame and a busier ISP clock, and about five interrupts per frame between rkcif and rkisp. The result is `docs/images/camera-fullframe-720p-isp-crop-2026-09-22.png`: full 62° width, correct aspect. Cost: 21 fps rather than 30, and the whole 8-megapixel frame through the ISP on every frame. In `mediad` this is a change to `pin_sensor_mode` (pin 3280x2464, or do not pin), one extra selection ioctl on the capture node, and `quality` set to a 21 fps rung or the frame rate cap relaxed. The exposure loop is unaffected: both modes share the same line length, so exposure in lines means the same time. Full 4:3 is also available as 1280x960 if a consumer wants the vertical field too; `duck-detect` letterboxes its input so it does not care about aspect, but the `quality` ladder names only 16:9 sizes.

2. Add the sensor's 2x2 binned 1640x1232 mode to the vendor driver. The driver on the board is exactly the one Armbian merged in `armbian/linux-rockchip` PR 313 (05/01/2025, identical in the rkr4.1, rkr5.1 and rkr7.2 branches): a cherry-pick from Joshua Riek's tree that replaced the mainline-derived driver, which did not work with Rockchip's CSI and ISP stack, with a Rockchip-style one carrying only the 1920x1080 and 3280x2464 tables. The driver it replaced is still in that repository's history at the PR's base commit `6693dbcacb`, and it has the 1640x1232 binned register table (`mode_1640_1232_regs`) plus a 640x480 one. Transplanting that table into the current driver's `supported_modes` is the job. It gives the full field of view at 30 fps, four times the light per pixel, and a quarter of the ISP load, then the ISP crops to 1640x923 and scales to 1280x720. The driver is built into the kernel (`CONFIG_VIDEO_IMX219=y`), and the PR discussion records that Rockchip camera drivers must be built in or the CSI, CIF and ISP bring-up order breaks, so this is a kernel rebuild through the Armbian build framework with a patch, not a module swap. Armbian's `rockchip-rk3588.conf` now builds the vendor kernel from `rk-6.1-rkr7.2`; the driver is the same there. The right long-term answer; the most work.

3. A wider lens. IMX219 modules are sold with 120° and 160° lenses. That widens the crop's field of view without touching software, at the cost of distortion that the intrinsics do not model, and it still throws away 66 % of the sensor and its light. A stopgap, not a fix.

Two things explained later the same evening, which supersede the darkness observations above:

- Sensor controls set before streaming starts are lost. The driver's `s_stream` rewrites the whole mode register table on start, and that table sets `INTEG_TIME` to 500 lines (`0x015A/0x015B = 0x01F4`), so an exposure written with `v4l2-ctl` before the stream begins is overwritten. Analogue gain survives because the table does not touch it. This is `armbian/linux-rockchip` issue 452, closed as not-a-bug because the 3A engine rewrites exposure every frame; `mediad::exposure` reasserts its controls periodically for the same reason. For hand tests, set exposure while the stream is running, and set it to a different value first: the V4L2 control framework only calls the driver when the cached value changes, so writing the value that is already cached from a previous run is a silent no-op while the sensor sits at the table default. `snap.sh` writes each control twice for this reason. The same applies to `mediad::exposure`'s periodic reassert: rewriting the values it already holds does nothing at the driver after a stream restart, so the port should nudge the value or toggle it. Every full resolution "darker than 1080p" reading above was taken at 500 lines, not the value logged.
- The light reaching the sensor is low, and the ISP then compresses it further. Raw Bayer captured straight off the CIF node (`/dev/video0`, `RG10`) decodes as a 40-bit little-endian stream, ten bits per pixel, four pixels per five bytes, row padded to 4352 bytes; the MIPI-style layout with a trailing low-bits byte is wrong for this driver and produces plausible but grainy pictures with mismatched greens. Correctly decoded, in the lit room on 22/09/2026 the sensor gave (black level 64, full scale 1023, controls set while streaming):

| Exposure lines | Analogue gain | Raw mean | Raw p99 | Saturated |
|---|---|---|---|---|
| 600 | 1x | 71 | 85 | 0 % |
| 4095 | 1x | 108 | 203 | 0 % |
| 600 | 11x | 140 | 288 | 0 % |
| 4095 | 11x | 555 | 1023 | 15 % |

  Signal scales linearly with both controls, so the sensor and the driver are behaving. The level is low for a lit room: at the longest exposure (77 ms) and unity gain the brightest percentile sits at 20 percent of full scale. Part of that is the light itself, warm and yielding a blue channel at half the green, which drags a green-weighted mean down; part may be the lens or something in front of it. Under the daemon's ceilings of 1200 lines and 11x the raw signal would sit around 15 percent linear, which with a real gamma curve is a usable if noisy picture. The ISP then makes a dim frame dimmer: with no 3A engine writing parameters it runs on bare driver defaults with no gamma or gains, so ISP output luma is not a measure of exposure until `rkaiq_3A_server` runs with an ISP39 tuning file. Radxa's own tracker says the same of the NX5 with this sensor (`radxa/meta-rockchip` issues 25 and 29): dark and green until the 3A server and the IQ file were installed, then normal.
- The two gain controls are one control. In this driver `V4L2_CID_ANALOGUE_GAIN` and `V4L2_CID_GAIN` share a handler that treats the value as total gain times 256 and splits it itself: up to 2728 (10.66x) it is all analogue with digital at 1x, above that analogue is pinned at 10.66x and digital takes the rest, capped at 4095 (16x). The last write wins, and the V4L2 framework only calls the driver when a value changes, which is why hand tests that set both looked as if they were independent. The upstream `mediad::exposure` loop drives the two separately with analogue capped at 11x; on this driver it should drive a single total gain. One unexplained measurement remains: total gain 4096 (16x) read about 3.5x darker in raw than 2816 at the same exposure, when it should have read 1.5x brighter. Do not rely on the digital region above 10.66x until that is understood.
- White balance of the raw frames: grey-world gains of about 1.0 on red and 2.3 on blue. ffmpeg's `colorchannelmixer` caps at 2.0, so use `colorlevels` with per-channel input maxima instead. The demosaiced frames are `scratch/raw-best-720p-2026-09-22.png` and the full resolution one beside it.

## 5. Known gaps

- 3A tuning: done, section 6. What remains is that the tuning is a transplant of Radxa's RK3588 IMX219 calibration into the RK3576 schema, not a calibration of this module and lens on this ISP. Colour looks right by eye; nobody has put a grey card or a colour checker in front of it yet.
- Two kernel-side fixes would remove two of the section 6 workarounds: re-applying the control handler in the imx219 driver's `s_stream` (mainline does, this vendor driver does not, Armbian issue 452), and either turning `CONFIG_RT_GROUP_SCHED` off or accepting that nothing under systemd can create a real-time thread on this image. The second one will matter again for the robot's control loop.
- `mediad` porting: done 24/09/2026, see section 8. It runs with `--no-auto-exposure` because the engine owns exposure, and with a new `--full-frame` flag (on the `dukki` branch of `../microduck`) that pins 3280x2464 to match `SENSOR_MODE` and sets the 16:9 ISP crop. That gives the full 62° field at 21 fps, so the upstream intrinsics hold unchanged. The two-gain-controls item only affected the exposure loop, which is off. If the engine is ever run in the 1080p mode, `mediad` must be run without `--full-frame` and the intrinsics corrected for 39°.
- Which IMX219 module and lens is fitted, and its orientation in the head. Both are open items in `hardware.md` section 13.
- MPP, the GStreamer plugins and the udev rules: `setup-media.sh`, section 8. Upstream's `setup-gstreamer.sh` alone is not enough on the RK3576.
- Restarting `mediad` can, rarely, crash the vendor ISP driver. Seen once on 03/10/2026, during a `push-daemons.sh` that restarted robotd, tofd, mediad and expressd together: the kernel logged `rkisp-vir1: waiting on params stream off event timeout` as the old stream stopped, then a NULL pointer dereference in `isp_lsc_config` (from `rkisp_params_first_cfg_v39`) as the new one started, in mediad's `v4l2src0:src` thread. After it no frame arrives (`robotctl frame` times out) until a reboot; the ISP's state is unknown after a kernel oops, so reboot rather than restarting services. Four further `systemctl restart mediad` on an idle board did not reproduce it, so it is a race, most likely with the 3A engine still holding the ISP's parameter stream when the next stream starts. If it recurs, stopping `microduck-rkaiq` before `mediad` and starting it after is the first thing to try. Check with `dmesg | grep -c "Internal error"` after any deploy that includes `mediad`.
- Header pins 27 and 28 are now the camera bus. They can still carry other I2C devices at other addresses, but the "spare I2C" note in earlier `hardware.md` revisions no longer applies.

## 6. The 3A engine and the IMX219 tuning file

Installed 22/09/2026 by `scripts/compute/setup-camera-3a.sh`. Result: four consecutive 720p streams off the ISP main path, each converging within a second to a mean luma of 104 to 110 with U and V within 6 of neutral, under the same warm room light that gave luma 21 to 27 without the engine. The frame is `docs/images/isp-3a-720p-2026-09-22.png`. After a reboot the service was active 45 s into boot with no help, and the engine cost 1 to 3 percent of one core while streaming 720p at 21 fps (a brief 9 percent at stream start) and 0.8 percent of memory.

### What runs

`rkaiq_3A_server` is Rockchip's user-space 3A engine: it reads statistics from `rkisp-statistics`, runs exposure, white balance, colour correction, shading, gamma, noise reduction and the rest, and writes the results into `rkisp-input-params` every frame and the exposure into the sensor. Without it the ISP runs on bare driver defaults, hence the dark green frames of section 4. The RK3576 build is `camera-engine-rkaiq` 6.8.0-rk3576 (AIQ v6.0x8.0, "ISP HW ver 39") from Radxa's `rk3576-bookworm` pool at `radxa-repo.github.io`; the setup script downloads it, or uses a copy placed in `camera/`. It needs `libdrm2`, which the image did not have. Its own options are `--silent` and nothing else: it hard-codes `/etc/iqfiles/` and picks the file named `<sensor>_<module>_<lens>.json` from the overlay's `rockchip,camera-module-name` and `-lens-name`, here `imx219_rpi-camera-v2_default.json`.

The service is `microduck-rkaiq.service`, not the deb's `rkaiq_3A.service`, which is disabled: the vendor unit starts at sysinit before udev has made the camera nodes and backgrounds the engine behind a pipe. `/usr/local/sbin/microduck-rkaiq.sh` waits up to a minute for the rkisp and imx219 media devices, pins the sensor mode, starts the resync helper below, and `exec`s the engine with the shim below preloaded. The engine reads the sensor format once when it prepares, so consumers must stream the pinned mode: `SENSOR_MODE` in `/etc/default/microduck-rkaiq`, default `3280x2464` (section 4a's full frame; `1920x1080` for the 30 fps crop). Change it and `systemctl restart microduck-rkaiq`. Streaming stays possible as `duck`; the engine holds the ISP's statistics and params nodes, not the main path.

### The tuning file

Radxa ships no IMX219 tuning for the RK3576's ISP. The engine validates the file against a per-ISP schema: RK356x files are `scene_isp21`, RK3588 files `scene_isp30`, RK3576 files `scene_isp39`, and the two IMX219 files the user found (`imx219_rpi-camera-v2_default.json` and `imx219_RADXA-CAMERA-8M_default.json`, from `rockchip-iqfiles-rk3588` 0.2.6) are isp30 and are refused. `camera/make-imx219-iq.py` builds an isp39 file by taking the engine's own S5K4H5YB file as the skeleton (same 3280x2464 array and 1.12 µm pitch) and transplanting what is a property of the IMX219 module rather than of the ISP: the register mapping for gain and exposure, black level per ISO, the white balance light sources and colour temperature line, the colour correction matrices per illuminant, and the lens shading tables for 3280x2464. The field mapping came from the one module Radxa tuned in both schemas, the IMX415 Radxa Camera 4K, by diffing its isp30 and isp39 files. Exposure loop, gamma, noise reduction and sharpening keep the skeleton's values. The generated file is checked in as `camera/imx219_rpi-camera-v2_default.json` (2 MB) so the setup script needs neither source.

Two things in the sensor description were changed from Radxa's numbers after watching the engine run. The engine writes three sensor controls every frame: `V4L2_CID_VBLANK`, `V4L2_CID_ANALOGUE_GAIN` and `V4L2_CID_EXPOSURE`, and never `V4L2_CID_GAIN`. On this driver the analogue control accepts 256 to 2816 (1x to 11x) and, as section 4 found, the two gain controls are one total-gain handler anyway. Radxa's file declares 11x analogue times 16x digital, 170x total; the exposure loop's own clamp is the product, 176x, so in any scene wanting more than 11x it asked for gains the range check rejected (`AEC:E:GAIN OUT OF RANGE`), stopped writing gain, saw a dark frame, and stayed at maximum for good. The file now declares 11x analogue, no sensor digital gain, up to 4x ISP digital gain, and an exposure route of time to 30 ms, then analogue gain to 11x, then ISP gain to 4x. The 30 ms cap is the skeleton's and suits a moving robot.

### Two workarounds the kernel makes necessary

Both are small, both are in `camera/`, and both would go away with a kernel change.

1. `rkaiq-sched-shim.c`, preloaded into the engine. The engine creates its statistics thread with `pthread_attr_setinheritsched(PTHREAD_EXPLICIT_SCHED)` and `SCHED_RR` priority 20. This kernel has `CONFIG_RT_GROUP_SCHED=y`, systemd enables the cgroup v2 `cpu` controller (`DefaultCPUAccounting=yes`), and under that combination a process in any cgroup but the root one gets `EPERM` from `sched_setscheduler`, root or not. glibc then fails the `pthread_create`, the engine carries on without its statistics thread, and the symptom is subtle: the file loads, colour is corrected, `rk_aiq_uapi2_sysctl_start success`, but exposure sits at the 3 ms start value forever and the log has no error. Found by `strace`: `sched_setscheduler(tid, SCHED_RR, [20]) = -1 EPERM` followed by the thread's exit; confirmed by moving the shell into the root cgroup (`echo $$ > /sys/fs/cgroup/cgroup.procs`), after which `xc:isp_3a_stats` appeared and exposure converged. The shim makes the inherit-sched call a no-op so threads inherit `SCHED_OTHER`; the engine keeps up with 21 fps regardless. `RKAIQ_ALLOW_RT=1` passes the call through. Disabling the `cpu` controller at runtime fails with `EBUSY` once systemd has populated it; `DefaultCPUAccounting=no` in `/etc/systemd/system.conf` plus a reboot should also work and was not tried. This limitation applies to every service on the image, so it is the first thing to check when a real-time control loop refuses its priority.

2. `rkaiq-stream-resync.c`, run beside the engine. The vendor imx219 driver rewrites exposure and gain from its mode table in `s_stream` but does not re-apply the V4L2 control values (Armbian issue 452), and the V4L2 core skips the driver when a written value equals the cached one. The engine's exposure converges to the 30 ms cap and then never changes, so after the first stream stop and restart the sensor sits at the table's exposure while the engine believes it is at 1575 lines: the second stream was 3x darker and, before the gain fix above, the loop then spiralled to maximum gain. The helper subscribes to `RKISP_V4L2_EVENT_STREAM_START` on the params node (`V4L2_EVENT_PRIVATE_START + 1`, not in the shipped headers) and 300 ms and 1000 ms after each start writes exposure and analogue gain to value minus one and back, which forces the driver call. `snap.sh` used the same trick by hand; with the service running its `--isp` mode now leaves the controls alone. The proper fix is one line in the driver, `__v4l2_ctrl_handler_setup()` after the mode table write, which mainline has.

### Checking it

`bash setup-camera-3a.sh --check` prints the package, the tuning file, both units, the engine's cgroup and threads, the helper, and the last log lines. While a stream runs the threads must include `xc:isp_3a_stats`; if `xc:isp_sof_poll` is there and that one is not, the shim is not loaded. `journalctl -u microduck-rkaiq` shows `resync: control ... re-applied` twice per stream start and should show no `GAIN OUT OF RANGE`. Two log lines are noise: `CAMHW:E:failed to set hdr mode 0` (the engine tries Rockchip's private `RKMODULE_*` ioctls, which this mainline-derived driver does not have, and copes) and the kernel's `vblank need >= 1000us if isp work in online, cur 685 us` (the driver fixes VBLANK at 36 lines; the ISP proc file reports no frame loss at 21 fps).

Investigation trail, for the next person with a "the engine runs but does nothing" problem: engine debug logging is `persist_camera_engine_log=0xffffffff4` in the environment (module mask then level); `strace -f -e trace=ioctl` on the engine shows which nodes it actually dequeues from (a healthy engine dequeues `rkisp-statistics` every frame); `/proc/rkisp-vir1` lists which ISP modules are on and the online frame count; the engine's C hardware layer is public in `khadas/external_camera_engine_rkaiq` under `rkaiq/hwi_c/` (`aiq_CamHwBase.c`, `aiq_stream.c`), which is how the `SCHED_RR` request was found.

## 8. `mediad`

`mediad` is the camera daemon. The monitor's camera block asks it for single raw frames over `/run/mediad/media.sock`, and it also serves the WebRTC video and console. Three pieces get it running on this board.

1. `scripts/compute/setup-media.sh`, on the board as root, with upstream's `setup-gstreamer.sh` copied beside it. It runs the upstream script (Debian GStreamer 1.26, Pollen's plugin bundle with `webrtcsink` and `mpph264enc`, udev rules for `/dev/mpp_service` and `/dev/rga`), then fixes three things for the RK3576:
   - MPP and RGA from Radxa's `rk3576-bookworm` pool, pinned by sha256. Upstream installs the same version numbers from the bullseye pool, which predates the RK3576. That build registers `mpph264enc`, then every frame times out in the kernel (`rk_vcodec: ... processing time out`, `rkvenc_soft_reset: safe reset failed`) and 60 frames encode to 45 bytes. `mpi_enc_test` fails the same way, so it is not GStreamer. The library has no RK3576 or `vepu510` strings; the `rk3576-bookworm` build has 56.
   - `/dev/dma_heap/*` to the `video` group. The RK3576 MPP build allocates from there, and the plugin only registers `mpph264enc` if that works. As a non-root user without it, `mpph265enc` registers but `mpph264enc` does not, and `mediad` quietly falls back to `x264enc`. Found by `strace`: the only `EACCES` is `openat("/dev/dma_heap/system")`. GStreamer's registry cache keeps the missing element, so retest with a fresh `GST_REGISTRY`.
   - `/dev/camera-main`, a udev link to the ISP main path by name. `mediad` defaults to `/dev/video0`, which is an rkcif raw node here; the main path was `video11`.
2. The `--full-frame` change to `mediad` (`mediad/src/pipeline.rs` `pin_full_frame`, `camera.rs` `SensorMode::FULL_FRAME_16_9`). It shells out to `media-ctl` for the sensor mode, as upstream does, and to `v4l2-ctl` for the crop selection, top row 310. The crop is fatal if it fails, because without it the ISP squashes 4:3 into 16:9. `v4l2src` refuses `framerate=30/1` from the full array (`not-negotiated`) rather than slowing down, so the flag also caps the rung's rate at 21.
3. `push-daemons.sh mediad` builds it against upstream's `cross-sysroot.sh` sysroot (kept in `~/.microduck/duck-aarch64-sysroot`) and installs a drop-in, `mediad.service.d/rk3576.conf`, with `--camera-device /dev/camera-main --full-frame --no-auto-exposure --rotate 0`. The rotation is the mount angle that the monitor and the web console turn the picture back by. Upstream defaults to 90 for Pollen's head, which showed this camera on its side. Set `CAMERA_ROTATE` when pushing once the camera sits in its final head mount (`hardware.md` section 13 leaves that orientation open).

Verified 24/09/2026: capture holds 21.1 fps, the monitor's camera block answers a frame in 27 ms, `media.video` publishes the family calibration (`fx` 1062 at 1280x720), and real camera frames encoded by `mpph264enc` as the `mediad` user decode back to a correct full-field picture (105 frames, 1.4 MB). `webrtcsink` logs `Bitrate handling is not supported yet for mpph264enc` at startup; that comes from Pollen's plugin bundle and should appear on the upstream board too. The WebRTC console on port 8080 has not been tried in a browser yet.

## 7. Sources

- CM4-NANO-A schematic, `docs/datasheets/CM4-NANO-A_SchDoc.pdf`, sheet 1: CSI connector nets, R20 and R21.
- Waveshare CM4-NANO-A wiki: "CM4-NANO only uses CAM0".
- `radxa-pkg/radxa-overlays`, `arch/arm64/boot/dts/rockchip/overlays/radxa-cm4-rpi-cm4-io-radxa-camera-8m-219-cam0.dts` and `-cam1.dts`.
- `radxa/kernel` branch `linux-6.1-stan-rkr5.1`, `rk3576-radxa-cm4-rpi-cm4-io.dts`: I2C0 M1 carries the Pi IO board's EMC2301 fan controller and PCF85063 RTC.
- Armbian `boot.cmd` on the board: `overlays` and `user_overlays` handling.
- `/boot/config-6.1.115-vendor-rk35xx` on the board.
- Upstream `microduck/docs/project/media-bringup.md`, `scripts/setup-rkaiq.sh`, `mediad/src/pipeline.rs` (`find_sensor`, `pin_sensor_mode`).
- Radxa pools: `https://radxa-repo.github.io/rk3576-bookworm/pool/main/c/camera-engine-rkaiq/` (engine), `rockchip-iqfiles-rk3588` 0.2.6 and `rockchip-iqfiles-rk3576` 0.2.6 (tuning files, isp30 and isp39).
- `khadas/external_camera_engine_rkaiq` on GitHub, `rkaiq/hwi_c/aiq_CamHwBase.c` and `aiq_stream.c`: stream start order, `SCHED_RR` on the statistics thread, `mNoReadBack` ("isOnline") logic.
- `armbian/linux-rockchip` issue 452 (exposure reset on stream start) and pull 313 (the imx219 driver), `radxa/meta-rockchip` issues 25 and 29 (dark green without rkaiq).
- Board: `/proc/rkisp-vir1`, `/proc/rkcif-mipi-lvds1`, `/sys/fs/cgroup/cgroup.subtree_control`, `/boot/config-6.1.115-vendor-rk35xx` (`CONFIG_RT_GROUP_SCHED=y`).
