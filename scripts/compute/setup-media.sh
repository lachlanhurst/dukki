#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel), as root. The GStreamer and Rockchip media stack
# mediad needs, adjusted for the RK3576. Safe to re-run; no reboot.
#
#   scp scripts/compute/setup-media.sh ../microduck/scripts/setup-gstreamer.sh root@microduck.local:/tmp/
#   ssh root@microduck.local bash /tmp/setup-media.sh
#
# 1. Runs upstream's setup-gstreamer.sh (copied alongside this script): Debian's GStreamer, Pollen's
#    plugin bundle in /usr/local/lib/gstreamer-1.0 (webrtcsink, mpph264enc) and the udev rule that
#    gives /dev/mpp_service and /dev/rga to the video group.
# 2. Replaces the MPP and RGA userspace it installs. Upstream takes them from Radxa's bullseye pool,
#    whose librockchip-mpp1 1.5.0-1 predates the RK3576: the encoder registers, then every frame
#    times out in the kernel ("rk_vcodec: ... processing time out", "rkvenc_soft_reset: safe reset
#    failed") and a 60 frame encode writes 45 bytes. Radxa's rk3576-bookworm pool carries builds
#    with the same version strings and RK3576 support, so dpkg sees no upgrade and nothing will
#    replace them, but a later run of upstream's script alone would not either: it skips the
#    install when the packages are present.
# 3. Names the ISP main path /dev/camera-main. Video node numbers follow probe order (on this board
#    rkcif takes video0 to video10 and the main path is video11), and mediad's default /dev/video0
#    is an rkcif raw node here.
# 4. Gives /dev/dma_heap/* to the video group. The RK3576 MPP build allocates its buffers there,
#    and the plugin registers mpph264enc only if that works, so a non-root mediad (upstream's rule
#    covers mpp_service and rga only) finds mpph265enc but no mpph264enc, and silently falls back to
#    x264enc. GStreamer's registry cache remembers the missing element: test with a fresh
#    GST_REGISTRY after changing this.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POOL=https://radxa-repo.github.io/rk3576-bookworm/pool/main
# Package, pool path, sha256 (from the pool's Packages index, 24/09/2026).
DEBS=(
  "librockchip-mpp1 m/mpp/librockchip-mpp1_1.5.0-1_arm64.deb 6a00cd23eb8ac59bc0e338ad0f88d61757a16c95d183b2983c86535e2a0fd378"
  "librockchip-vpu0 m/mpp/librockchip-vpu0_1.5.0-1_arm64.deb 6219b9b5ea93b8fc8d3fe473335cd5b44d658c152c28718c9728b33476f6bc88"
  "librga2 libr/librga/librga2_2.2.0-1_arm64.deb ca4f18666f6c5d5290c7e41e5901350ecf76530f24364e37b81fa6be4ab5f344"
)
STAMP=/var/lib/microduck/rk3576-media.sha256
RULE=/etc/udev/rules.d/99-microduck-camera.rules
RULE_CONTENT='SUBSYSTEM=="video4linux", ATTR{name}=="rkisp_mainpath", SYMLINK+="camera-main"
SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"'

say() { printf '>> %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "run as root"
[[ -f "${HERE}/setup-gstreamer.sh" ]] || die "copy ../microduck/scripts/setup-gstreamer.sh alongside this script"

# 1. Upstream's stack.
sh "${HERE}/setup-gstreamer.sh" | sed 's/\x1b\[[0-9;]*m//g' | grep -E '^(==>|warning|error)' || true

# 2. RK3576 builds of MPP and RGA. The stamp records what was installed, so a re-run is a no-op
#    and a changed pin reinstalls.
want="$(printf '%s\n' "${DEBS[@]}" | awk '{print $3}')"
if [[ -f "$STAMP" && "$(cat "$STAMP")" == "$want" ]]; then
  say "RK3576 MPP and RGA already installed"
else
  work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
  for entry in "${DEBS[@]}"; do
    read -r pkg path sum <<<"$entry"
    curl -fsSL -o "${work}/${pkg}.deb" "${POOL}/${path}"
    echo "${sum}  ${work}/${pkg}.deb" | sha256sum -c --quiet - || die "${pkg}: sha256 mismatch"
  done
  dpkg -i "${work}"/*.deb >/dev/null
  install -d "$(dirname "$STAMP")"
  printf '%s\n' "$want" > "$STAMP"
  say "installed RK3576 builds of librockchip-mpp1, librockchip-vpu0 and librga2"
fi

# 3 and 4. /dev/camera-main and /dev/dma_heap.
if [[ -f "$RULE" && "$(cat "$RULE")" == "$RULE_CONTENT" ]]; then
  say "${RULE} already in place"
else
  printf '%s\n' "$RULE_CONTENT" > "$RULE"
  chmod 644 "$RULE"
  say "installed ${RULE}"
fi
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux --subsystem-match=dma_heap
udevadm settle
ls -l /dev/camera-main || die "no /dev/camera-main: is the camera overlay active (setup-camera.sh)?"
ls -l /dev/dma_heap/

# 5. Prove the encoder: 60 frames of 720p should be hundreds of kilobytes, not a header.
out="$(mktemp)"
GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0 gst-launch-1.0 -q videotestsrc num-buffers=60 \
  ! video/x-raw,width=1280,height=720 ! mpph264enc ! h264parse ! filesink location="$out"
size="$(stat -c %s "$out")"; rm -f "$out"
(( size > 100000 )) || die "mpph264enc wrote ${size} bytes for 60 frames; check dmesg for rk_vcodec timeouts"
say "mpph264enc: ${size} bytes for 60 frames of 720p"
say "done"
