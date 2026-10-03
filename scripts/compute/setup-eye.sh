#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Enables SPI1 on the CM4-NANO-A header (MOSI on
# pin 19) for the WS2812-type eye LED by compiling eye/microduck-nano-a-spi1.dts and installing
# it as an Armbian user overlay, then installs the udev rule that names the spidev node
# /dev/spidev-eye and gives it to the robot group. Safe to re-run. Does not reboot; the overlay
# takes effect on the next boot.
#
#   scp -r scripts/compute/setup-eye.sh scripts/compute/eye duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-eye.sh           # install (sudo prompts), then reboot
#   ssh -t duck@microduck.local sudo bash setup-eye.sh --check   # after the reboot: check the node
#
#   DRY_RUN=1 bash setup-eye.sh    compile and test-apply only, as any user, writes nothing
#
# Why the symlink: spidev numbers follow the spi aliases and probe order, so the eye is not
# guaranteed to stay /dev/spidev1.0. The rule matches the controller by its address instead.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=microduck-nano-a-spi1
SRC="${HERE}/eye/${NAME}.dts"
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
# SPI1's register base on the RK3576, as the device tree names it.
CONTROLLER=2ad00000.spi
RULE=/etc/udev/rules.d/99-robot-spidev-eye.rules
RULE_CONTENT="SUBSYSTEM==\"spidev\", KERNELS==\"${CONTROLLER}\", SYMLINK+=\"spidev-eye\", GROUP=\"robot\", MODE=\"0660\""
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  echo "--- SPI1 controller status:"
  tr -d '\0' < "/proc/device-tree/spi@${CONTROLLER%.spi}/status" 2>/dev/null; echo
  echo "--- spidev node and symlink:"
  ls -l /dev/spidev* 2>/dev/null || echo "(no /dev/spidev*: overlay not active)"
  ls -l "$(readlink -f /dev/spidev-eye 2>/dev/null || echo /dev/spidev-eye)" 2>/dev/null \
    || echo "(no /dev/spidev-eye: udev rule missing)"
}

if [[ "${1:-}" == "--check" ]]; then check; exit 0; fi

# 1. Preconditions.
for t in dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler)"; done
[[ -f "$SRC" ]] || die "no ${SRC}; copy the eye/ directory alongside this script"
FDT="$(sed -n 's/^fdtfile=//p' "$ENV_FILE")"
BASE="/boot/dtb/${FDT}"
[[ -f "$BASE" ]] || die "base device tree ${BASE} not found"

# 2. Compile, then test-apply against the tree this board boots so an unresolved label fails
#    here and not silently at boot.
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
dtc -@ -I dts -O dtb -q -o "${WORK}/${NAME}.dtbo" "$SRC"
fdtoverlay -i "$BASE" -o "${WORK}/merged.dtb" "${WORK}/${NAME}.dtbo" || die "overlay does not apply to ${BASE}"
say "compiled ${NAME}.dtbo and applied it to $(basename "$BASE") without error"

if [[ "$DRY_RUN" == "1" ]]; then say "dry run: nothing installed"; exit 0; fi
[[ $EUID -eq 0 ]] || die "run as root to install (sudo), or DRY_RUN=1 to only compile"

# 3. Install the overlay.
install -d "$USER_DIR"
if [[ -f "${USER_DIR}/${NAME}.dtbo" ]] && cmp -s "${WORK}/${NAME}.dtbo" "${USER_DIR}/${NAME}.dtbo"; then
  say "${USER_DIR}/${NAME}.dtbo already up to date"
else
  install -m 0644 "${WORK}/${NAME}.dtbo" "${USER_DIR}/${NAME}.dtbo"
  say "installed ${USER_DIR}/${NAME}.dtbo"
fi

# 4. Name it in armbianEnv.txt. Append to an existing user_overlays= line, or add one.
if grep -qE "^user_overlays=.*(^|[ =])${NAME}([ ]|$)" "$ENV_FILE"; then
  say "user_overlays already lists ${NAME}"
elif grep -qE '^user_overlays=' "$ENV_FILE"; then
  sed -i -E "s/^(user_overlays=.*)$/\1 ${NAME}/" "$ENV_FILE"
  say "added ${NAME} to user_overlays"
else
  echo "user_overlays=${NAME}" >> "$ENV_FILE"
  say "added user_overlays=${NAME}"
fi
grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE"

# 5. The /dev/spidev-eye symlink, owned by the robot group so eyed needs no root.
if [[ -f "$RULE" ]] && [[ "$(cat "$RULE")" == "$RULE_CONTENT" ]]; then
  say "${RULE} already in place"
else
  printf '%s\n' "$RULE_CONTENT" > "$RULE"
  chmod 644 "$RULE"
  say "installed ${RULE}"
  udevadm control --reload-rules
fi

say "done. Reboot, then: sudo bash $(basename "$0") --check"
