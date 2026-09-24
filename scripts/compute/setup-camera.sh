#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Enables the IMX219 on the CM4-NANO-A's CSI
# connector by compiling camera/microduck-nano-a-imx219.dts and installing it as an Armbian user
# overlay. Safe to re-run. Does not reboot; the overlay takes effect on the next boot.
#
#   scp -r scripts/compute/setup-camera.sh scripts/compute/camera duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-camera.sh       # install (sudo prompts), then reboot
#   ssh duck@microduck.local bash setup-camera.sh --check       # after the reboot: did it probe?
#
#   DRY_RUN=1 bash setup-camera.sh    compile and test-apply only, as any user, writes nothing to /boot
#
# Why a user overlay: Armbian loads /boot/overlay-user/<name>.dtbo for each word in user_overlays=
# in /boot/armbianEnv.txt, with no prefix, and outside the kernel package's overlay directory, so a
# kernel update does not remove it. If an overlay fails to apply at boot, boot.cmd reloads the
# original device tree, so a bad overlay costs the camera and not the boot.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=microduck-nano-a-imx219
SRC="${HERE}/camera/${NAME}.dts"
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  echo "--- installed user overlays:"
  ls -l "$USER_DIR" 2>/dev/null || echo "(none)"
  echo "--- imx219 in dmesg (needs root to read on this image):"
  (dmesg 2>/dev/null || sudo -n dmesg 2>/dev/null || true) | grep -i -E 'imx219|rkisp|rkcif|csi2' | tail -n 20 || true
  echo "--- video and media nodes:"
  ls -l /dev/video* /dev/media* 2>/dev/null || echo "(none: overlay not active, camera not connected, or probe failed)"
  if command -v media-ctl >/dev/null && ls /dev/media* >/dev/null 2>&1; then
    for m in /dev/media*; do
      echo "--- $m entities:"
      media-ctl -d "$m" -p 2>/dev/null | grep -E '^- entity' || echo "(cannot read $m; try with sudo or add yourself to the video group)"
    done
  fi
  echo "--- capture test (works as a member of the video group, no sudo):"
  echo "    media-ctl -d /dev/media0 --set-v4l2 '\"m00_f_imx219 6-0010\":0[fmt:SRGGB10_1X10/1920x1080]'"
  echo "    MP=\$(media-ctl -d /dev/media1 -p | awk '/entity .*rkisp_mainpath/{f=1} f&&/device node name/{print \$4; exit}')"
  echo "    v4l2-ctl -d \$MP --set-fmt-video=width=1280,height=720,pixelformat=NV12 --stream-mmap=4 --stream-count=30 --stream-to=/tmp/cam.nv12"
  echo "  (the sensor is on the rkcif media device, the main path on the rkisp one; node numbers change between boots)"
  echo "--- next: bash setup-camera-3a.sh (as root) installs the 3A engine; until then ISP frames are dark and green"
}

if [[ "${1:-}" == "--check" ]]; then check; exit 0; fi

# 1. Preconditions.
KVER="$(uname -r)"
[[ "$KVER" == *vendor* ]] || die "kernel ${KVER} is not the Armbian vendor kernel; rkcif, rkisp and the CSI D-PHY only exist there"
INC="/usr/src/linux-headers-${KVER}/include"
[[ -f "${INC}/dt-bindings/pinctrl/rockchip.h" ]] || die "no dt-bindings headers under ${INC}; apt install linux-headers-vendor-rk35xx"
for t in cpp dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler gcc)"; done
[[ -f "$SRC" ]] || die "no ${SRC}; copy the camera/ directory alongside this script"
FDT="$(sed -n 's/^fdtfile=//p' "$ENV_FILE")"
BASE="/boot/dtb/${FDT}"
[[ -f "$BASE" ]] || die "base device tree ${BASE} not found"

# 2. Compile, then test-apply against the tree this board boots so an unresolved label fails
#    here and not silently at boot.
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
cpp -nostdinc -I"$INC" -undef -x assembler-with-cpp "$SRC" -o "${WORK}/${NAME}.pre.dts"
dtc -@ -I dts -O dtb -q -o "${WORK}/${NAME}.dtbo" "${WORK}/${NAME}.pre.dts"
fdtoverlay -i "$BASE" -o "${WORK}/merged.dtb" "${WORK}/${NAME}.dtbo" || die "overlay does not apply to ${BASE}"
say "compiled ${NAME}.dtbo and applied it to $(basename "$BASE") without error"

if [[ "$DRY_RUN" == "1" ]]; then say "dry run: nothing installed"; exit 0; fi
[[ $EUID -eq 0 ]] || die "run as root to install (sudo), or DRY_RUN=1 to only compile"

# 3. Install.
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

say "done. Reboot, then: bash $(basename "$0") --check"
say "expect an imx219 line in dmesg, /dev/media0 and several /dev/video* nodes (rkisp_mainpath, rkisp_selfpath, rkisp_rawrd*, stream_cif_mipi_id*)"
