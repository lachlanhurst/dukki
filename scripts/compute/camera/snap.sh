#!/usr/bin/env bash
# Take one picture with the head camera and save it into scratch/ as PNG. Runs on the Mac.
#
#   scripts/compute/camera/snap.sh                 # raw sensor frame, demosaiced here: the true picture
#   scripts/compute/camera/snap.sh --isp           # what the robot's pipeline sees: 720p off the ISP main path
#   EXPOSURE=2400 GAIN=1024 scripts/compute/camera/snap.sh
#   HOST=duck@192.168.20.35 scripts/compute/camera/snap.sh   # when microduck.local does not resolve
#
# Defaults: full 3280x2464 sensor frame, exposure 4095 lines (77 ms), total gain 1536 (6x), controls set
# while streaming because the driver rewrites exposure on stream start. With the 3A engine running on the
# board (setup-camera-3a.sh) the --isp picture is auto exposed and the controls are left alone unless
# EXPOSURE or GAIN is given. The raw path bypasses the ISP,
# which without a 3A engine produces dark green frames; the frame is unpacked from Rockchip's packed
# 10-bit layout, black level subtracted, grey-world white balanced and gamma encoded with ffmpeg.
# Output: scratch/snap-<timestamp>-720p.png (full width, 16:9) and -full.png (raw path only).
# Needs ffmpeg and clang on the Mac; v4l-utils on the board (present).
set -euo pipefail
HOST="${HOST:-duck@microduck.local}"
MANUAL=0; [[ -n "${EXPOSURE:-}${GAIN:-}" ]] && MANUAL=1   # caller asked for fixed controls
EXPOSURE="${EXPOSURE:-4095}"; GAIN="${GAIN:-1536}"
ISP=0; [[ "${1:-}" == "--isp" ]] && ISP=1
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$ROOT/scratch"; mkdir -p "$OUT/.bin"
STAMP="$(date +%Y%m%d-%H%M%S)"
command -v ffmpeg >/dev/null || { echo "ffmpeg not found on the Mac (brew install ffmpeg)"; exit 1; }

# Remote side: find the nodes by name (numbers change between boots), stream, set controls mid-stream,
# write the last frame to stdout.
REMOTE='
set -e
ISP='"$ISP"'; E='"$EXPOSURE"'; G='"$GAIN"'; MANUAL='"$MANUAL"'
node() { media-ctl -d "$1" -p | awk -v e="$2" "/entity/{f=index(\$0,e)>0} f&&/device node name/{print \$4; exit}"; }
SD=$(node /dev/media0 imx219); CIF=$(node /dev/media0 stream_cif_mipi_id0); MP=$(node /dev/media1 rkisp_mainpath)
ENT=$(media-ctl -d /dev/media0 -p | grep -o "m00_._imx219 [0-9]*-0010" | head -1)
media-ctl -d /dev/media0 --set-v4l2 "\"$ENT\":0[fmt:SRGGB10_1X10/3280x2464]" >/dev/null
if [ "$ISP" = 1 ]; then
  v4l2-ctl -d $MP --set-selection=target=crop,top=310,left=0,width=3280,height=1845 >/dev/null 2>&1
  v4l2-ctl -d $MP --set-fmt-video=width=1280,height=720,pixelformat=UYVY >/dev/null 2>&1
  DEV=$MP; FS=$((1280*720*2))
else
  DEV=$CIF; FS=$((4352*2464))
  v4l2-ctl -d $DEV --set-fmt-video=width=3280,height=2464,pixelformat=RG10 >/dev/null 2>&1
fi
rm -f /tmp/snap.bin
# With the 3A engine running (see setup-camera-3a.sh) the ISP path is auto exposed: leave the controls
# alone unless the caller set EXPOSURE or GAIN, and stream long enough for the exposure loop to settle.
AUTO=0; if [ "$ISP" = 1 ] && [ "$MANUAL" = 0 ] && pgrep -x rkaiq_3A_server >/dev/null; then AUTO=1; fi
N=45; [ "$AUTO" = 1 ] && N=84
timeout 25 v4l2-ctl -d $DEV --stream-mmap=4 --stream-count=$N --stream-to=/tmp/snap.bin >/dev/null 2>&1 &
if [ "$AUTO" = 0 ]; then
  # wait until frames are actually flowing before touching the controls: the driver rewrites exposure to its
  # table default on stream start, so a write that lands earlier is lost
  sleep 1.5
  # Two writes each: V4L2 only calls the driver when the cached value changes, and the cache may already hold
  # the value we want from a previous run while the sensor sits at the table default the stream start wrote.
  # (v4l2-ctl exits non-zero on a harmless VIDIOC_SUBDEV_S_CLIENT_CAP warning, hence the || true.)
  for v in $((E>0 ? E-1 : E+1)) $E; do v4l2-ctl -d $SD --set-ctrl=exposure=$v >/dev/null 2>&1 || true; done
  for v in $((G>256 ? G-1 : G+1)) $G; do v4l2-ctl -d $SD --set-ctrl=analogue_gain=$v >/dev/null 2>&1 || true; done
fi
wait
[ "$AUTO" = 1 ] && echo "3A engine running: auto exposure, ended at exposure $(v4l2-ctl -d $SD --get-ctrl=exposure 2>/dev/null | grep -v CLIENT | awk "{print \$2}") lines, analogue gain $(v4l2-ctl -d $SD --get-ctrl=analogue_gain 2>/dev/null | grep -v CLIENT | awk "{print \$2}")/256" >&2
tail -c $FS /tmp/snap.bin; rm -f /tmp/snap.bin
'
if [[ $ISP = 1 && $MANUAL = 0 ]]; then echo ">> capturing from $HOST (ISP main path, auto exposure if the 3A engine is running, else exposure $EXPOSURE lines, total gain $GAIN)"
else echo ">> capturing from $HOST (exposure $EXPOSURE lines, total gain $GAIN, $([[ $ISP = 1 ]] && echo ISP main path || echo raw sensor))"; fi
if [[ $ISP = 1 ]]; then
  ssh -o ConnectTimeout=8 "$HOST" "$REMOTE" > "$OUT/.snap.uyvy"
  ffmpeg -hide_banner -loglevel error -y -f rawvideo -pixel_format uyvy422 -video_size 1280x720 -i "$OUT/.snap.uyvy" -frames:v 1 "$OUT/snap-$STAMP-720p.png"
  rm -f "$OUT/.snap.uyvy"
else
  [[ -x "$OUT/.bin/unpack10" ]] || clang -O2 -o "$OUT/.bin/unpack10" "$ROOT/scripts/compute/camera/unpack10.c"
  ssh -o ConnectTimeout=8 "$HOST" "$REMOTE" > "$OUT/.snap.raw"
  MEANS="$("$OUT/.bin/unpack10" "$OUT/.snap.raw" 3280 2464 4352 "$OUT/.snap.raw16")"
  echo ">> $MEANS"
  read -r RI BI <<<"$(echo "$MEANS" | awk '{r=$4-64; g=(($6-64)+($8-64))/2; b=$10-64; printf "%.4f %.4f", r/g, b/g}')"
  VF="colorlevels=rimax=$RI:gimax=1:bimax=$BI,eq=gamma=2.2"
  ffmpeg -hide_banner -loglevel error -y -f rawvideo -pixel_format bayer_rggb16le -video_size 3280x2464 -i "$OUT/.snap.raw16" -frames:v 1 -vf "$VF" "$OUT/snap-$STAMP-full.png"
  ffmpeg -hide_banner -loglevel error -y -f rawvideo -pixel_format bayer_rggb16le -video_size 3280x2464 -i "$OUT/.snap.raw16" -frames:v 1 -vf "$VF,crop=3280:1845:0:310,scale=1280:720" "$OUT/snap-$STAMP-720p.png"
  rm -f "$OUT/.snap.raw" "$OUT/.snap.raw16"
fi
echo ">> saved:"; ls -1 "$OUT"/snap-"$STAMP"-*.png
open "$OUT/snap-$STAMP-720p.png" 2>/dev/null || true
