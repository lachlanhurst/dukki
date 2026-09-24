#!/usr/bin/env bash
# Runs ON the board as root, from microduck-rkaiq.service (installed by setup-camera-3a.sh). Starts
# Rockchip's 3A engine for the head camera with the two workarounds this kernel needs, and keeps the
# engine as the service's main process. See docs/camera-setup.md section 7.
#
#   1. Waits for the camera media graph: udev creates the nodes some seconds into boot and the engine
#      exits if it finds no ISP.
#   2. Pins the sensor mode. The engine reads the sensor format once at start and sizes the ISP for it,
#      so consumers must use this mode (full frame, cropped and scaled in the ISP, see camera-setup.md
#      4a). Change SENSOR_MODE in /etc/default/microduck-rkaiq to run the 1920x1080 crop instead.
#   3. Starts rkaiq-stream-resync, which re-applies the sensor's exposure controls after every stream
#      start because the driver resets the registers but not the V4L2 cache (issue 452).
#   4. Starts rkaiq_3A_server with rkaiq-sched-shim.so preloaded: its statistics thread wants SCHED_RR,
#      which CONFIG_RT_GROUP_SCHED denies to any systemd service, and without that thread the engine
#      never runs its 3A loop.
set -uo pipefail
[[ -r /etc/default/microduck-rkaiq ]] && . /etc/default/microduck-rkaiq
SENSOR_MODE="${SENSOR_MODE:-3280x2464}"
SHIM="${SHIM:-/usr/local/lib/rkaiq-sched-shim.so}"
RESYNC="${RESYNC:-/usr/local/bin/rkaiq-stream-resync}"
ENGINE="${ENGINE:-/usr/bin/rkaiq_3A_server}"

node() { media-ctl -d "$1" -p 2>/dev/null | awk -v e="$2" '/^- entity/{f=index($0,e)>0} f&&/device node name/{print $4; exit}'; }

ISPM=""; CIFM=""
for _ in $(seq 1 60); do
  for m in /dev/media*; do
    [[ -e "$m" ]] || continue
    top="$(media-ctl -d "$m" -p 2>/dev/null)"
    grep -q 'rkisp-input-params' <<<"$top" && ISPM="$m"
    grep -q 'imx219' <<<"$top" && CIFM="$m"
  done
  [[ -n "$ISPM" && -n "$CIFM" ]] && break
  sleep 1
done
[[ -n "$ISPM" && -n "$CIFM" ]] || { echo "microduck-rkaiq: no rkisp and imx219 media devices after 60 s (overlay missing, or no camera)"; exit 1; }

PARAMS="$(node "$ISPM" rkisp-input-params)"
SD="$(node "$CIFM" imx219)"
ENT="$(media-ctl -d "$CIFM" -p | grep -o 'm00_._imx219 [0-9]*-0010' | head -1)"
[[ -n "$PARAMS" && -n "$SD" && -n "$ENT" ]] || { echo "microduck-rkaiq: could not resolve nodes (params=$PARAMS sensor=$SD entity=$ENT)"; exit 1; }

media-ctl -d "$CIFM" --set-v4l2 "\"$ENT\":0[fmt:SRGGB10_1X10/$SENSOR_MODE]" || echo "microduck-rkaiq: warning: could not pin sensor mode $SENSOR_MODE"
echo "microduck-rkaiq: sensor $ENT ($SD) pinned to $SENSOR_MODE, ISP params node $PARAMS"

"$RESYNC" "$PARAMS" "$SD" &

LD_PRELOAD="$SHIM" exec "$ENGINE"
