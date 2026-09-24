# Microduck Unitree: compute setup

How to take a Radxa CM4 (RK3576) out of the box, put it on the Waveshare CM4-NANO-A carrier, write Armbian to its eMMC from a Mac, and end up with a board on the WiFi that you can reach as `microduck.local` over SSH with a key. This is done once per module. The hardware reasoning behind these choices is in `hardware.md` sections 4.1 to 4.5. Installing the robot software on top of the base OS is out of scope here and follows `microduck/scripts/setup-board.sh`.

Date: 19/09/2026. Written against Armbian 26.8.2 (Debian 13 "trixie", vendor kernel 6.1.115) for the `radxa-cm4-io` board, and macOS 26 on Apple silicon.

## 1. Sources

- Radxa CM4 downloads (loader, Radxa's own images, RKDevTool): https://docs.radxa.com/en/som/cm/cm4/download
- Radxa maskrom guide for the CM4: https://docs.radxa.com/en/som/cm/cm4/low-dev/rkdevtool_maskrom
- Radxa rkdeveloptool guide (generic): https://docs.radxa.com/en/rock3/rock3a/low-level-dev/rkdeveloptool
- rkdeveloptool source: https://github.com/rockchip-linux/rkdeveloptool
- RK3576 loaders: https://github.com/radxa/rkbin, branch `develop-v2026.01`, `bin/rk35/`
- Armbian for the Radxa CM4 IO board: https://www.armbian.com/radxa-cm4-io/
- Armbian first-boot presets: https://docs.armbian.com/user-guide/autoconfig/
- Armbian's wizard itself, `/usr/lib/armbian/armbian-firstlogin` inside the image. Several statements below come from reading it, not from the docs.
- Waveshare CM4-NANO-A wiki and schematic (`docs/datasheets/CM4-NANO-A_SchDoc.pdf`), for the BOOT switch and USB-C wiring.

## 2. The facts that shape this procedure

- The NANO-A's BOOT switch grounds the Pi nRPIBOOT position. On the Radxa CM4 that forces the RK3576 into USB maskrom mode and routes its download port to the carrier's USB-C. Maskrom is a Rockchip USB protocol. It does not present as a disk, so nothing mounts and Armbian Imager, Balena Etcher and Finder cannot see it. Only `rkdeveloptool` (Mac and Linux) or RKDevTool (Windows) can write the eMMC this way. The Raspberry Pi `rpiboot` procedure on Waveshare's wiki does not apply.
- Homebrew has no `rkdeveloptool` formula and the upstream autotools build fails with the autotools found on most Macs. The setup script compiles the sources directly with clang.
- On this module `rkdeveloptool ld` reports `Maskrom` even after the loader is running, and a second `db` upload reports "failed". The reliable test that the loader is resident is `rkdeveloptool rfi`, which prints the eMMC size. The loader is lost on any reset.
- Armbian only applies the first-boot presets and creates the user account from its first-login wizard, which runs on the first interactive login as root. Booting alone does not run it. The two `PRESET_*_KEY` variables are URLs that the wizard downloads with curl. Given literal key text, curl fails and the wizard exits before setting any password. The scripts here therefore put the public key straight into the image instead.
- The wizard sources the preset file with bash and exits silently if any line fails a syntax check. The flash script quotes every value with `printf %q` so passwords with quotes or dollar signs are safe.
- The image has systemd-resolved but no mDNS responder. Installing `avahi-daemon` and setting the hostname makes the board reachable as `microduck.local`.
- A wizard run that fails part way can leave `/etc/netplan/30-wifis-dhcp.yaml` without the WiFi passphrase. WiFi then works until the next boot. `board-finish-setup.sh` rewrites that file.

## 3. What you need

Hardware:

- Radxa CM4 module (2 GB, 16 or 32 GB eMMC) on the Waveshare CM4-NANO-A carrier. The header need not be soldered for this procedure.
- USB-C cable to a Mac. The carrier draws its 5 V from the same cable while flashing.
- Optional: a USB to 3.3 V serial adapter on header pins 8 and 10 (UART0, 1500000 baud) for a console if the network does not come up.

Software on the Mac:

- Homebrew, Xcode command line tools (clang, git), python3, xz. The setup script installs `libusb` and `e2fsprogs` from Homebrew.
- An SSH key pair. The scripts default to `~/.ssh/id_ed25519.pub`.

## 4. Scripts

All in `scripts/compute/`. They keep their downloads and builds in `RK_FLASH_DIR`, default `~/.microduck/rk-flash`, outside the repository.

| Script | Runs on | Does |
|---|---|---|
| `setup-flash-tools.sh` | Mac | Installs Homebrew deps, builds `rkdeveloptool` at a pinned commit, downloads the pinned RK3576 loader and Armbian image, checks sha256, decompresses. Re-runnable. |
| `flash-cm4.sh` | Mac | Clones the image, injects the first-boot preset file and your public key (for root and `/etc/skel`), verifies the board over USB, writes the eMMC, resets the board. `DRY_RUN=1` prepares the image without touching the board. |
| `board-finish-setup.sh` | Board, as root | Completes setup by hand when the wizard fails: fixes the netplan WiFi file, sets both passwords, creates the user with Armbian's group list (including `dialout`), sets the time zone and hostname, installs avahi for `microduck.local`, removes the preset file. Optional `DISABLE_PASSWORD_AUTH=1`. |
| `setup-head-i2c.sh` | Board, as root | Installs the I2C8 overlay (header pins 3 and 5, 400 kHz) for the head ToF and BMI088, and the udev rule that names the bus `/dev/i2c-pihat` for `tofd`. `--check` scans the bus after a reboot. |
| `setup-audio.sh` | Board, as root | Installs the SAI2 overlay and `/etc/asound.conf` for the head speaker and microphone. See `audio-setup.md`. |
| `push-daemons.sh` | Mac | Cross-builds Microduck daemons from `../microduck` with `cargo zigbuild`, and installs them on the board through `install-daemons.sh`. See section 5.8. |
| `setup-media.sh` | Board, as root | GStreamer, the RK3576 builds of MPP and RGA, and udev rules for the VPU, `dma_heap` and `/dev/camera-main`. Run before pushing `mediad`. See `camera-setup.md` section 8. |
| `install-daemons.sh` | Board, as root | Installs a staged set of binaries, units and sysusers files in upstream's `/opt/robot/daemon/current` layout. Run by `push-daemons.sh`. |

Pinned versions live at the top of `setup-flash-tools.sh`. Change them together and re-run this whole procedure before trusting the result.

## 5. Procedure

### 5.1 Prepare the Mac

```
scripts/compute/setup-flash-tools.sh
```

About 400 MB downloads and a few seconds of compiling. Ends with "Ready".

### 5.2 Put the module into maskrom mode

1. Seat the module on the carrier. Check the three board-to-board connectors are fully home.
2. BOOT switch on.
3. Connect USB-C to the Mac. The carrier powers up in maskrom mode.
4. Confirm:

```
cd ~/.microduck/rk-flash && ./rkdeveloptool ld
```

Run it from that directory: rkdeveloptool writes a `log/` folder into the current directory. Expect one line with `Vid=0x2207,Pid=0x350e` and `Maskrom`. If nothing is listed, check the switch was on before power was applied, try another cable or port, and look for vendor 8711 (decimal for 0x2207) in `system_profiler SPUSBDataType`.

### 5.3 Write the eMMC

```
WIFI_SSID='your network' WIFI_KEY='your passphrase' \
USER_NAME='duck' USER_PASSWORD='choose one' ROOT_PASSWORD='choose one' \
scripts/compute/flash-cm4.sh
```

Optional variables: `WIFI_COUNTRY` (AU), `SSH_PUBKEY`, `TIMEZONE` (Australia/Melbourne), `LOCALE` (en_AU.UTF-8), `RK_FLASH_DIR`.

The script prints the preset file with secrets redacted, uploads the loader if `rfi` fails, prints the eMMC size, writes about 2.3 GB, and resets the board. Expect several minutes for the write. It deletes the prepared image afterwards because it holds the passphrase and passwords in clear text.

If it reports the board not responding after the loader upload, power cycle with the BOOT switch still on and run it again.

### 5.4 First boot

1. BOOT switch off.
2. Power cycle. Leave it for a few minutes. Armbian resizes the root filesystem and joins the WiFi from the presets.
3. Find the board's IP on the router. Its hostname is `radxa-cm4-io`.
4. Log in as root first. This is what triggers the wizard:

```
ssh root@<ip>
```

Your key is already authorised for root, so no password is needed. If you have no key, the default root password is `1234`. The wizard runs on its own, reports the network configuration, sets the root password, creates the user, sets locale and time zone, deletes the preset file and restarts sshd. It kills other root shells while it does this. Let it finish.

5. Log in as the user:

```
ssh duck@<ip>
```

The key works here too, because `useradd` copied `/etc/skel/.ssh/authorized_keys` into the new home.

### 5.5 Name the board

The image has no mDNS responder, so at this point the board answers only by IP. Give it a name and install avahi, as root on the board:

```
hostnamectl set-hostname microduck
sed -i 's/radxa-cm4-io/microduck/g' /etc/hosts
apt update && apt install -y avahi-daemon
```

From then on the board is reachable from any Mac on the same network with:

```
ssh duck@microduck.local
```

macOS resolves `.local` names natively and the name follows the board if the router changes its IP. `board-finish-setup.sh` does the same thing (variable `BOARD_HOSTNAME`, default `microduck`). A DHCP reservation on the router is a sensible fallback for networks that block multicast DNS. Verified 19/09/2026 on the first module.

If `microduck.local` stops resolving and the board is otherwise fine, look at `journalctl -u avahi-daemon` for "Host name conflict, retrying with microduck-2". Seen 22/09/2026: the WiFi IPv6 address changed while avahi was probing its name at boot, avahi mistook its own re-announcement for another host, and answered only as `microduck-2.local` afterwards. An `scp` or `ssh` to the old name then sits in mDNS resolution for a long time rather than failing. Reach the board by IP, then either restart avahi or apply the durable fix, which `board-finish-setup.sh` now does as well:

```
sudo sed -i -E 's/^#?use-ipv6=.*/use-ipv6=no/; s/^#?publish-aaaa-on-ipv4=.*/publish-aaaa-on-ipv4=no/' /etc/avahi/avahi-daemon.conf
sudo systemctl restart avahi-daemon
```

Setting `use-ipv6=no` on its own is not enough. It was seen again on 22/09/2026 with only that setting. Avahi still published the IPv6 addresses as AAAA records over IPv4, so the same churn caused the same conflict. With `publish-aaaa-on-ipv4=no` as well, avahi announces only the IPv4 A record, and that does not change at boot. `grep -E 'use-ipv6|publish-aaaa' /etc/avahi/avahi-daemon.conf` should show both set to `no`. After a restart, the journal should show no `Registering new address record for fe80::` or `fd..` lines.

The script also installs `avahi-name-guard.timer`. Every minute it compares avahi's announced name with `hostname` and puts it back over D-Bus if a conflict has renamed it. Any reset it makes is logged with the tag `avahi-name-guard`, so `journalctl -t avahi-name-guard` shows whether conflicts are still happening.

The hostname itself is unaffected; only avahi's announced name changes.

Optional, on the Mac, in `~/.ssh/config`:

```
Host microduck
    HostName microduck.local
    User duck
```

which shortens the command to `ssh microduck` and applies to `scp` and `rsync` as well.

### 5.6 If the wizard fails

Symptoms: the user account does not exist, or root still accepts `1234`. Still as root on the board, see where it stops:

```
bash -n /root/.not_logged_in_yet; echo "syntax: $?"
bash -x /usr/lib/armbian/armbian-firstlogin 2>&1 | tail -30
```

Then finish by hand rather than fighting it:

```
scp scripts/compute/board-finish-setup.sh root@<ip>:
ssh root@<ip> bash board-finish-setup.sh
```

It reads the preset file for the values. If that file is already gone, pass them in the environment as documented at the top of the script.

### 5.7 Harden and verify

Once `ssh duck@microduck.local` works without a password:

```
ssh root@microduck.local DISABLE_PASSWORD_AUTH=1 bash board-finish-setup.sh
```

or by hand, add `PasswordAuthentication no` in `/etc/ssh/sshd_config.d/50-keys-only.conf` and `systemctl restart ssh`. Do not do this before key login is confirmed.

Reboot once and confirm the board comes back on WiFi as `microduck.local`. That proves the netplan file carries the passphrase. If it does not come back, use the UART0 console on header pins 8 and 10, or reflash.

Checks worth recording per module:

```
cat /proc/device-tree/model          # Radxa CM4 IO
hostname                             # microduck
uname -r                             # 6.1.115-vendor-rk35xx
lsblk                                # mmcblk0 about 29 GB
iw dev                               # wlan0 present
nmcli dev status 2>/dev/null || networkctl
id duck                              # includes dialout
```

### 5.8 Microduck daemons and the monitor

The firmware's own daemons are installed by hand for now, not through `updaterd`. The board is not a signed dev board yet, and some daemons cannot run on this hardware until they are ported. `push-daemons.sh` cross-builds from the `../microduck` checkout on the Mac, stages each binary with its upstream unit and sysusers file, and runs `install-daemons.sh` on the board as root:

```
scripts/compute/push-daemons.sh                       # robotctl and tofd
ROBOTD_FAKE=1 scripts/compute/push-daemons.sh robotd  # robotd with no servo bus
scripts/compute/push-daemons.sh mediad                # camera; after setup-media.sh
```

The layout is the one upstream's `install.sh` produces: binaries under `/opt/robot/daemon/hand/bin`, `current` pointing at `hand`, units copied into `/etc/systemd/system`, and `/usr/local/bin/robotctl` linked through `current`. The upstream unit files therefore run unchanged. `install-daemons.sh` refuses to run if `current` points at a release the updater installed. It creates the `robot` group and adds `duck` to it, which is what lets `robotctl` reach the daemon sockets. Log in again after the first run.

`ROBOTD_FAKE=1` adds a drop-in, `/etc/systemd/system/robotd.service.d/fake.conf`, that runs `robotd --fake --no-policy`: a 50 Hz loop against a robot made of nothing, holding the startup pose. Push `robotd` again without the variable to remove it once the bridge exists. With no gamepad the safety deadman logs "intents went stale" once a second, which is expected.

Then, as `duck`, in a terminal of at least 120 by 45:

```
robotctl monitor      # p pad, t ToF, c camera, q quits
robotctl health
```

State on 24/09/2026, firmware 0.15.0 (`a9ec4b2`):

| Monitor block | Daemon | State |
|---|---|---|
| Robot, joints, loop rate, power | `robotd --fake` | Working. Loop at 50.0 Hz with no missed ticks under `SCHED_OTHER`. Battery and motor figures are the fake robot's; CPU temperature is real |
| ToF (`t`) | `tofd` | Working. VL53L8CX found on `/dev/i2c-pihat` at 0x29 and ranging at 15 Hz about 2.6 s after start, upstream code unchanged |
| Camera (`c`) | `mediad` | Working. Frame answered in 27 ms, capture at 21 fps from the full sensor array. Needs `setup-media.sh` first; see `camera-setup.md` section 8 |
| Pad (`p`) | `padd` | Not installed. Needs a paired Bluetooth gamepad |

`robotctl health` also reports `updaterd` and `configd` as unavailable; neither is installed.

## 6. Known gaps

- The image boots Radxa's CM4 IO board device tree, not a Pi-carrier tree. Header functions the NANO-A needs are enabled afterwards by their own scripts, each installing an Armbian user overlay: CSI with `setup-camera.sh` (`camera-setup.md`), I2C8 with `setup-head-i2c.sh`, and SAI2 with `setup-audio.sh` (`audio-setup.md`). UART7 for the bridge link has no overlay yet.
- Armbian Imager 2.0 can write a customised image to a microSD card and the RK3576 falls through to SD when the eMMC is blank. That is a workable alternative for a first look, followed by `armbian-install` to copy onto the eMMC, but it is not what this document describes and has not been tried here.
