#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Sets up the outputs expressd drives:
#
#   microduck-nano-a-spi1     SPI1, MOSI on header pin 19, for the WS2812-type eye LED
#   microduck-nano-a-pwm-fan  PWM1 channel 2 on header pin 11, for the head fan's MOSFET
#
# Compiles both overlays from express/ and installs them as Armbian user overlays, then installs
# two udev rules: one names the spidev node /dev/spidev-eye and gives it to the robot group, the
# other gives the fan's PWM sysfs files to the robot group. Safe to re-run. Does not reboot; the
# overlays take effect on the next boot.
#
#   scp -r scripts/compute/setup-express.sh scripts/compute/express duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-express.sh           # install (sudo prompts), then reboot
#   ssh -t duck@microduck.local sudo bash setup-express.sh --check   # after the reboot: check the node
#
#   DRY_RUN=1 bash setup-express.sh    compile and test-apply only, as any user, writes nothing
#
# Why the rules match controller addresses: spidev and pwmchip numbers follow probe order, so the
# eye is not guaranteed to stay /dev/spidev1.0 nor the fan pwmchip1.
#
# The PWM rule runs on `change` as well as `add` because exporting a channel (which expressd does
# at start) creates its pwmN directory and announces it as a change event on the chip; the
# files in it are root-only until the rule regroups them.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAMES=(microduck-nano-a-spi1 microduck-nano-a-pwm-fan)
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
# Register bases on the RK3576, as the device tree names them: SPI1, and PWM1 channel 2.
SPI=2ad00000.spi
PWM=2add2000.pwm
SPI_RULE=/etc/udev/rules.d/99-robot-spidev-eye.rules
SPI_RULE_CONTENT="SUBSYSTEM==\"spidev\", KERNELS==\"${SPI}\", SYMLINK+=\"spidev-eye\", GROUP=\"robot\", MODE=\"0660\""
PWM_RULE=/etc/udev/rules.d/99-robot-pwm-fan.rules
PWM_RULE_CONTENT="SUBSYSTEM==\"pwm\", KERNELS==\"${PWM}\", ACTION==\"add|change\", RUN+=\"/bin/sh -c 'chgrp -R robot /sys%p && chmod -R g+w /sys%p'\""
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  echo "--- SPI1 and fan PWM controller status:"
  tr -d '\0' < "/proc/device-tree/spi@${SPI%.spi}/status" 2>/dev/null; echo
  tr -d '\0' < "/proc/device-tree/pwm@${PWM%.pwm}/status" 2>/dev/null; echo
  echo "--- spidev node and symlink:"
  ls -l /dev/spidev* 2>/dev/null || echo "(no /dev/spidev*: overlay not active)"
  ls -l "$(readlink -f /dev/spidev-eye 2>/dev/null || echo /dev/spidev-eye)" 2>/dev/null \
    || echo "(no /dev/spidev-eye: udev rule missing)"
  echo "--- fan pwmchip:"
  ls -ld /sys/devices/platform/${PWM}/pwm/pwmchip*/export 2>/dev/null \
    || echo "(no pwmchip for ${PWM}: overlay not active)"
}

# Write a udev rule if it differs. Prints whether it changed anything.
install_rule() {
  local path="$1" content="$2"
  if [[ -f "$path" ]] && [[ "$(cat "$path")" == "$content" ]]; then
    say "${path} already in place"
  else
    printf '%s\n' "$content" > "$path"
    chmod 644 "$path"
    say "installed ${path}"
    RELOAD=1
  fi
}

if [[ "${1:-}" == "--check" ]]; then check; exit 0; fi

# 1. Preconditions.
for t in dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler)"; done
for n in "${NAMES[@]}"; do
  [[ -f "${HERE}/express/${n}.dts" ]] || die "no express/${n}.dts; copy the express/ directory alongside this script"
done
FDT="$(sed -n 's/^fdtfile=//p' "$ENV_FILE")"
BASE="/boot/dtb/${FDT}"
[[ -f "$BASE" ]] || die "base device tree ${BASE} not found"

# 2. Compile, then test-apply against the tree this board boots so an unresolved label fails
#    here and not silently at boot. Applied together, as the bootloader will.
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
dtbos=()
for n in "${NAMES[@]}"; do
  dtc -@ -I dts -O dtb -q -o "${WORK}/${n}.dtbo" "${HERE}/express/${n}.dts"
  dtbos+=("${WORK}/${n}.dtbo")
done
fdtoverlay -i "$BASE" -o "${WORK}/merged.dtb" "${dtbos[@]}" || die "overlays do not apply to ${BASE}"
say "compiled ${NAMES[*]} and applied them to $(basename "$BASE") without error"

if [[ "$DRY_RUN" == "1" ]]; then say "dry run: nothing installed"; exit 0; fi
[[ $EUID -eq 0 ]] || die "run as root to install (sudo), or DRY_RUN=1 to only compile"

# 3. Install the overlays, and name each in armbianEnv.txt (append to user_overlays=, or add it).
install -d "$USER_DIR"
for n in "${NAMES[@]}"; do
  if [[ -f "${USER_DIR}/${n}.dtbo" ]] && cmp -s "${WORK}/${n}.dtbo" "${USER_DIR}/${n}.dtbo"; then
    say "${USER_DIR}/${n}.dtbo already up to date"
  else
    install -m 0644 "${WORK}/${n}.dtbo" "${USER_DIR}/${n}.dtbo"
    say "installed ${USER_DIR}/${n}.dtbo"
  fi
  if grep -qE "^user_overlays=.*(^|[ =])${n}([ ]|$)" "$ENV_FILE"; then
    say "user_overlays already lists ${n}"
  elif grep -qE '^user_overlays=' "$ENV_FILE"; then
    sed -i -E "s/^(user_overlays=.*)$/\1 ${n}/" "$ENV_FILE"
    say "added ${n} to user_overlays"
  else
    echo "user_overlays=${n}" >> "$ENV_FILE"
    say "added user_overlays=${n}"
  fi
done
grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE"

# 4. The udev rules, so expressd needs no root.
RELOAD=0
install_rule "$SPI_RULE" "$SPI_RULE_CONTENT"
install_rule "$PWM_RULE" "$PWM_RULE_CONTENT"
[[ "$RELOAD" == "1" ]] && udevadm control --reload-rules

say "done. Reboot, then: sudo bash $(basename "$0") --check"
