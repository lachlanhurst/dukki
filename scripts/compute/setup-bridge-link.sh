#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Enables UART7 on the CM4-NANO-A header (pins 16
# and 18) for the servo bridge link by compiling bridge-link/microduck-nano-a-uart7.dts and
# installing it as an Armbian user overlay, makes sure no login console runs on the port, and
# installs a boot service that moves the port's interrupt off CPU 0 (below). Safe to re-run.
# Does not reboot; the overlay takes effect on the next boot.
#
#   scp -r scripts/compute/setup-bridge-link.sh scripts/compute/bridge-link duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-bridge-link.sh          # install, then reboot
#   ssh -t duck@microduck.local sudo bash setup-bridge-link.sh --check  # after the reboot
#
#   DRY_RUN=1 bash setup-bridge-link.sh    compile and test-apply only, as any user, writes nothing
#
# robotd runs as root, so the port needs no udev rule. The device name comes from the tree's
# serial7 alias, /dev/ttyS7 on the vendor kernel; --check prints what it really is.
#
# Why the boot service: the vendor 8250 driver receives UART7 in interrupt mode (its DMA receive
# lost frames when tried), and the 16-byte FIFO interrupts at 8 bytes, leaving about 40 us at
# 2 Mbps before it overruns. By default the interrupt lands on CPU 0, which takes nearly every
# other interrupt on the board, and that load grows with the camera, ISP and audio running. The
# service moves it to CPU 3, a Cortex-A53 that takes no others, leaving the A72s to the policy.
# On the bench the link missed about 3 to 5 replies in 6,000 at 100 Hz on either core, so this
# is insurance against a busy CPU 0 rather than a measured fix (hardware.md 5.5).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=microduck-nano-a-uart7
SRC="${HERE}/bridge-link/${NAME}.dts"
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
PORT=ttyS7
TUNE_CPU=3
TUNE_BIN=/usr/local/sbin/microduck-bridge-link-tune
TUNE_UNIT=/etc/systemd/system/microduck-bridge-link-tune.service
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  local node
  node="$(tr -d '\0' < /proc/device-tree/aliases/serial7 2>/dev/null || true)"
  echo "--- UART7 (${node:-no serial7 alias}) status:"
  [[ -n "$node" ]] && { tr -d '\0' < "/proc/device-tree${node}/status" 2>/dev/null; echo; }
  echo "--- port:"
  ls -l "/dev/${PORT}" 2>/dev/null || echo "(no /dev/${PORT}: overlay not active)"
  dmesg 2>/dev/null | grep -iE "${PORT}|serial7" | tail -3 || true
  echo "--- anything holding the port (expect nothing, or robotd once it runs):"
  fuser -v "/dev/${PORT}" 2>&1 || true
  echo "--- getty on ${PORT}: $(systemctl is-enabled "serial-getty@${PORT}.service" 2>/dev/null || echo none)"
  local irq
  irq="$(cat "/sys/class/tty/${PORT}/irq" 2>/dev/null || true)"
  echo "--- interrupt ${irq:-?} set to CPU $(cat "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null || echo ?) (want ${TUNE_CPU})"
  echo "--- receive errors: $(grep "^${PORT#ttyS}:" /proc/tty/driver/serial 2>/dev/null)"
}

if [[ "${1:-}" == "--check" ]]; then check; exit 0; fi

# 1. Preconditions.
for t in dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler)"; done
[[ -f "$SRC" ]] || die "no ${SRC}; copy the bridge-link/ directory alongside this script"
FDT="$(sed -n 's/^fdtfile=//p' "$ENV_FILE")"
BASE="/boot/dtb/${FDT}"
[[ -f "$BASE" ]] || die "base device tree ${BASE} not found"

# 2. Compile, then test-apply against the tree this board boots so an unresolved label (the
#    uart7 node or its uart7m0_xfer pin group) fails here and not silently at boot.
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

# 5. A login console on the link would answer the bridge's frames with a login prompt. Armbian
#    only runs one on its console port, but mask it explicitly so a changed default cannot.
if [[ "$(systemctl is-enabled "serial-getty@${PORT}.service" 2>/dev/null || true)" != "masked" ]]; then
  systemctl mask "serial-getty@${PORT}.service" >/dev/null 2>&1 || true
  say "masked serial-getty@${PORT}"
fi

# 6. The boot service that moves the port's interrupt off CPU 0.
TUNE_CONTENT="#!/bin/sh
# Installed by dukki scripts/compute/setup-bridge-link.sh. UART7's interrupt on CPU ${TUNE_CPU}, away
# from CPU 0 and its other interrupts, so the 16-byte receive FIFO is served before it overruns.
# The driver requests the interrupt when the port is first opened, and until then there is no
# affinity to set; once set it holds across later opens. stty opens it, at the link's settings.
stty -F /dev/${PORT} 2000000 raw -echo || exit 1
irq=\$(cat /sys/class/tty/${PORT}/irq) || exit 1
echo ${TUNE_CPU} > /proc/irq/\$irq/smp_affinity_list
exit 0"
UNIT_CONTENT="[Unit]
Description=Move the servo bridge link's UART interrupt off CPU 0
Before=robotd.service
After=systemd-udev-settle.service

[Service]
Type=oneshot
ExecStart=${TUNE_BIN}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target"
if [[ "$(cat "$TUNE_BIN" 2>/dev/null)" != "$TUNE_CONTENT" ]]; then
  printf '%s\n' "$TUNE_CONTENT" > "$TUNE_BIN"; chmod 755 "$TUNE_BIN"; say "installed ${TUNE_BIN}"
fi
if [[ "$(cat "$TUNE_UNIT" 2>/dev/null)" != "$UNIT_CONTENT" ]]; then
  printf '%s\n' "$UNIT_CONTENT" > "$TUNE_UNIT"; systemctl daemon-reload; say "installed ${TUNE_UNIT}"
fi
systemctl enable microduck-bridge-link-tune.service >/dev/null 2>&1
if [[ -e "/sys/class/tty/${PORT}/irq" ]]; then systemctl restart microduck-bridge-link-tune.service && say "applied the interrupt tuning now"; fi

say "done. Reboot, then: sudo bash $(basename "$0") --check"
