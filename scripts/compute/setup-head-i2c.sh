#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Enables I2C8 on the CM4-NANO-A header (pins 3
# and 5) for the head ToF and BMI088 by compiling head-i2c/microduck-nano-a-i2c8.dts and
# installing it as an Armbian user overlay, then installs the udev rule that names the bus
# /dev/i2c-pihat for tofd. Safe to re-run. Does not reboot; the overlay takes effect on the next
# boot.
#
#   scp -r scripts/compute/setup-head-i2c.sh scripts/compute/head-i2c duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-head-i2c.sh     # install (sudo prompts), then reboot
#   ssh -t duck@microduck.local sudo bash setup-head-i2c.sh --check   # after the reboot: scan the bus
#
#   DRY_RUN=1 bash setup-head-i2c.sh    compile and test-apply only, as any user, writes nothing
#
# Why the symlink: tofd opens /dev/i2c-pihat, then falls back to /dev/i2c-3. Bus numbers follow
# probe order, so I2C8 is not guaranteed to be /dev/i2c-8, and on this board /dev/i2c-3 is the
# CM4 IO tree's I2C3, not the header. The rule matches the controller by its address instead.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=microduck-nano-a-i2c8
SRC="${HERE}/head-i2c/${NAME}.dts"
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
# I2C8's register base on the RK3576, as the device tree names it.
CONTROLLER=2acb0000.i2c
RULE=/etc/udev/rules.d/99-robot-i2c-pihat.rules
RULE_CONTENT="SUBSYSTEM==\"i2c-dev\", KERNELS==\"${CONTROLLER}\", SYMLINK+=\"i2c-pihat\""
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  echo "--- I2C8 controller status:"
  tr -d '\0' < "/proc/device-tree/i2c@${CONTROLLER%.i2c}/status" 2>/dev/null; echo
  echo "--- bus and symlink:"
  ls -l /dev/i2c-pihat 2>/dev/null || echo "(no /dev/i2c-pihat: overlay not active or udev rule missing)"
  local dev
  dev="$(readlink -f /dev/i2c-pihat 2>/dev/null || true)"
  if [[ -c "$dev" ]] && command -v i2cdetect >/dev/null; then
    echo "--- scan of ${dev} (expect 0x29 for the ToF; 0x18 and 0x68 once the BMI088 is wired):"
    i2cdetect -y -r "${dev##*-}"
  fi
}

if [[ "${1:-}" == "--check" ]]; then check; exit 0; fi

# 1. Preconditions.
for t in dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler)"; done
[[ -f "$SRC" ]] || die "no ${SRC}; copy the head-i2c/ directory alongside this script"
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

# 5. The /dev/i2c-pihat symlink. Same file name as upstream setup-board.sh writes, so running
#    that later replaces this rule with one for the RK3566 and the ToF disappears: rerun this.
if [[ -f "$RULE" ]] && [[ "$(cat "$RULE")" == "$RULE_CONTENT" ]]; then
  say "${RULE} already in place"
else
  printf '%s\n' "$RULE_CONTENT" > "$RULE"
  chmod 644 "$RULE"
  say "installed ${RULE}"
fi

say "done. Reboot, then: sudo bash $(basename "$0") --check"
