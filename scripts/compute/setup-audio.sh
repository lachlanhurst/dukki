#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel). Enables SAI2 on the CM4-NANO-A header (pins 12,
# 35, 38 and 40) with a dummy-codec sound card for the head MAX98357A amplifier and INMP441
# microphone, by compiling audio/microduck-nano-a-sai2.dts and installing it as an Armbian user
# overlay, then installs audio/asound.conf as /etc/asound.conf so the default ALSA device is the
# new card with a softvol "Speaker" control. Safe to re-run. Does not reboot; the overlay takes
# effect on the next boot.
#
#   scp -r scripts/compute/setup-audio.sh scripts/compute/audio duck@microduck.local:
#   ssh -t duck@microduck.local sudo bash setup-audio.sh           # install (sudo prompts), then reboot
#   ssh -t duck@microduck.local bash setup-audio.sh --check        # after the reboot: card state
#   ssh -t duck@microduck.local bash setup-audio.sh --test         # play a test tone on the speaker
#
#   DRY_RUN=1 bash setup-audio.sh    compile and test-apply only, as any user, writes nothing
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=microduck-nano-a-sai2
SRC="${HERE}/audio/${NAME}.dts"
ASOUND_SRC="${HERE}/audio/asound.conf"
ASOUND=/etc/asound.conf
ENV_FILE=/boot/armbianEnv.txt
USER_DIR=/boot/overlay-user
CARD=microduck
DRY_RUN="${DRY_RUN:-0}"

say()  { printf '>> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

check() {
  echo "--- kernel: $(uname -r)"
  echo "--- fdtfile and overlays in ${ENV_FILE}:"
  grep -E '^(fdtfile|overlays|user_overlays)=' "$ENV_FILE" || true
  echo "--- SAI2 controller status:"
  tr -d '\0' < /proc/device-tree/sai@2a620000/status 2>/dev/null; echo
  echo "--- sound cards:"
  cat /proc/asound/cards
  grep -q "\[${CARD} *\]" /proc/asound/cards || echo "(no ${CARD} card: overlay not active, see dmesg | grep -iE 'sai|simple|asoc')"
  echo "--- ${ASOUND}:"
  [[ -f "$ASOUND" ]] && echo "installed" || echo "(missing)"
  echo "--- mixer controls on ${CARD} (Speaker appears after the first playback):"
  amixer -c "$CARD" scontrols 2>/dev/null || true
}

test_tone() {
  grep -q "\[${CARD} *\]" /proc/asound/cards || die "no ${CARD} card; run --check"
  # softvol creates its control on first open, so play a moment of silence before setting it.
  aplay -q -f S16_LE -r 48000 -c 2 -d 1 /dev/zero
  say "setting Speaker to 80%; adjust with: amixer sset Speaker 60%"
  amixer -q sset Speaker 80%
  say "playing a 440 Hz tone for 3 s on the default device"
  # Generated here rather than with speaker-test, whose long periods under a timeout played nothing.
  python3 -c '
import math, struct, sys
r = 48000
sys.stdout.buffer.write(b"".join(struct.pack("<h", int(12000 * math.sin(2 * math.pi * 440 * i / r))) for i in range(3 * r)))
' | aplay -q -f S16_LE -r 48000 -c 1 -t raw
  say "playing /usr/share/sounds/alsa/Front_Center.wav"
  aplay -q /usr/share/sounds/alsa/Front_Center.wav
}

case "${1:-}" in
  --check) check; exit 0 ;;
  --test)  test_tone; exit 0 ;;
esac

# 1. Preconditions.
for t in dtc fdtoverlay; do command -v "$t" >/dev/null || die "missing ${t} (apt install device-tree-compiler)"; done
command -v aplay >/dev/null || warn "aplay missing (apt install alsa-utils)"
[[ -f "$SRC" && -f "$ASOUND_SRC" ]] || die "missing files under ${HERE}/audio; copy the audio/ directory alongside this script"
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

# 5. ALSA defaults. Keep any existing file that is not ours.
if [[ -f "$ASOUND" ]] && cmp -s "$ASOUND_SRC" "$ASOUND"; then
  say "${ASOUND} already up to date"
else
  [[ -f "$ASOUND" ]] && cp "$ASOUND" "${ASOUND}.bak" && say "saved existing ${ASOUND} as ${ASOUND}.bak"
  install -m 0644 "$ASOUND_SRC" "$ASOUND"
  say "installed ${ASOUND}"
fi

say "done. Reboot, then: bash $(basename "$0") --check && bash $(basename "$0") --test"
