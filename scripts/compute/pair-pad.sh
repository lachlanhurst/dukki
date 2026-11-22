#!/usr/bin/env bash
# Runs on the board as root. Pairs a Bluetooth gamepad for padd, for when `robotctl pad pair` cannot:
# that goes through configd, which is not installed (compute-setup.md section 5.8).
#
#   scripts/compute/pair-pad.sh                       first "Wireless Controller" (a PS4 pad) it finds
#   scripts/compute/pair-pad.sh DC:AF:68:BB:38:26     that address
#   PAD_NAME="Xbox Wireless Controller" scripts/compute/pair-pad.sh
#
# Put the pad in pairing mode first (PS4: hold Share and PS until the light bar double-flashes). The
# script keeps one bluetoothctl session open with an agent registered for the whole attempt: a
# one-shot `bluetoothctl pair` has no agent, the pad's pairing request is refused ("Authentication
# attempt without agent") and the pad drops out of pairing mode. It pairs the moment the pad shows in
# the scan, then trusts and connects it; a trusted pad reconnects on PS from then on. It scans again
# after a failed attempt, for up to TIMEOUT seconds (default 300). Needs bluez (apt install bluez;
# systemctl enable --now bluetooth).
set -euo pipefail

MAC="${1:-}"
NAME="${PAD_NAME:-Wireless Controller}"
TIMEOUT="${TIMEOUT:-300}"
LOG="$(mktemp)"
FIFO="$(mktemp -u)"
mkfifo "${FIFO}"
bluetoothctl < "${FIFO}" > "${LOG}" 2>&1 &
BT=$!
exec 3> "${FIFO}"
cleanup() { echo "quit" >&3 2>/dev/null || true; exec 3>&-; sleep 1; kill "${BT}" 2>/dev/null || true; rm -f "${FIFO}" "${LOG}"; }
trap cleanup EXIT
say() { echo "$1" >&3; sleep "${2:-1}"; }
clean() { sed 's/\x1b\[[0-9;]*m//g' "${LOG}" | tail -n +"${from}"; }

say "agent on"
say "default-agent"
[[ -n "${MAC}" ]] && say "remove ${MAC}" 2
from=$(($(wc -l < "${LOG}") + 1))
say "scan on" 0
echo "scanning for ${MAC:-"\"${NAME}\""}; put the pad in pairing mode"
end=$((SECONDS + TIMEOUT))
asked=0
while (( SECONDS < end )); do
  sleep 1
  if (( asked == 0 )); then
    if [[ -z "${MAC}" ]]; then
      MAC="$(clean | grep -oE "NEW\] Device ([0-9A-F]{2}:){5}[0-9A-F]{2} ${NAME}" | head -1 | grep -oE "([0-9A-F]{2}:){5}[0-9A-F]{2}" || true)"
      [[ -n "${MAC}" ]] && echo "found ${NAME} at ${MAC}"
    fi
    if [[ -n "${MAC}" ]] && clean | grep -qE "(NEW|CHG)\] Device ${MAC}"; then
      say "scan off"; echo "pairing ${MAC}"; say "pair ${MAC}" 0; asked=1; t=${SECONDS}
    fi
  elif clean | grep -q "Pairing successful"; then
    say "trust ${MAC}"; say "connect ${MAC}" 8
    bluetoothctl info "${MAC}" | grep -E "Name|Paired|Trusted|Connected"
    exit 0
  elif clean | grep -q "Failed to pair" || (( SECONDS - t > 25 )); then
    echo "pairing failed ($(clean | grep -oE "Failed to pair.*" | tail -1)); scanning again"
    say "remove ${MAC}" 2; from=$(($(wc -l < "${LOG}") + 1)); say "scan on" 0; asked=0
  fi
done
echo "no pad paired in ${TIMEOUT} s" >&2
exit 1
