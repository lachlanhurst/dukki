#!/usr/bin/env bash
# Runs on the Mac. Cross-builds Microduck daemons from the firmware checkout, stages each binary
# with its upstream systemd unit and sysusers file, copies the stage to the board and runs
# install-daemons.sh there as root. A hand install for bring-up: unsigned, and not through
# updaterd, so the board does not need to be a dev board yet.
#
#   scripts/compute/push-daemons.sh                  robotctl and tofd (the default set)
#   scripts/compute/push-daemons.sh tofd             one daemon
#   ROBOTD_FAKE=1 scripts/compute/push-daemons.sh robotd   robotd with no servo bus (--fake)
#   scripts/compute/push-daemons.sh mediad           camera daemon; run setup-media.sh first
#   DRY_RUN=1 scripts/compute/push-daemons.sh        build and stage only
#
# Environment:
#   MICRODUCK_DIR  firmware checkout (default: ../microduck beside this repo)
#   BOARD          ssh target for the install, needs root (default: root@microduck.local)
#   OPERATOR       login added to the `robot` group on the board (default: duck)
#   CAMERA_ROTATE  mediad --rotate: how far the camera is mounted from upright, clockwise, 0, 90,
#                  180 or 270 (default: 0). Upstream's default is 90 for Pollen's head; the monitor
#                  and the web console both turn the picture back by this, so it must match the
#                  real mount or both show it on its side.
#
# Needs rustup's stable toolchain with the aarch64-unknown-linux-gnu target, cargo-zigbuild and
# zig (brew install rustup zig; cargo install cargo-zigbuild --locked). The .2.31 suffix pins the
# glibc floor, as upstream's `cargo board` alias does. The `tof` crate's C build prints
# "version '.2.31' in target triple ... is invalid" warnings while probing compiler flags; they
# are harmless, the vendored ST driver still links.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MICRODUCK_DIR="${MICRODUCK_DIR:-$(cd "${HERE}/../../../microduck" && pwd)}"
BOARD="${BOARD:-root@microduck.local}"
OPERATOR="${OPERATOR:-duck}"
DRY_RUN="${DRY_RUN:-0}"
ROBOTD_FAKE="${ROBOTD_FAKE:-0}"
CAMERA_ROTATE="${CAMERA_ROTATE:-0}"
TARGET=aarch64-unknown-linux-gnu
DAEMONS=("$@")
(( ${#DAEMONS[@]} )) || DAEMONS=(robotctl tofd)

say() { printf '>> %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# Homebrew's rustup does not put its proxies on PATH; use the toolchain directly if cargo is not.
if ! command -v cargo >/dev/null; then
  export PATH="${HOME}/.rustup/toolchains/stable-aarch64-apple-darwin/bin:${PATH}"
fi
command -v cargo-zigbuild >/dev/null || export PATH="${HOME}/.cargo/bin:${PATH}"
for t in cargo cargo-zigbuild zig; do command -v "$t" >/dev/null || die "missing ${t}"; done

# Binary name -> cargo package. The package directory also holds the binary's systemd/ files.
package_of() {
  case "$1" in
    robotctl) echo robotctl ;;
    tofd)     echo tof ;;
    robotd|padd|mediad) echo "$1" ;;
    *) die "unknown daemon ${1}" ;;
  esac
}

pkgs=()
for d in "${DAEMONS[@]}"; do pkgs+=(-p "$(package_of "$d")"); done

# mediad links GStreamer and padd links libudev, so those two build against the board's own
# Debian arm64 packages, unpacked by upstream's cross-sysroot.sh. Kept outside TMPDIR, which macOS
# clears, beside the flashing downloads.
if [[ " ${DAEMONS[*]} " == *" mediad "* || " ${DAEMONS[*]} " == *" padd "* ]]; then
  export DUCK_SYSROOT="${DUCK_SYSROOT:-${HOME}/.microduck/duck-aarch64-sysroot}"
  sh "${MICRODUCK_DIR}/scripts/cross-sysroot.sh" --check >/dev/null 2>&1 \
    || sh "${MICRODUCK_DIR}/scripts/cross-sysroot.sh" >/dev/null
  export PKG_CONFIG_SYSROOT_DIR="$DUCK_SYSROOT"
  export PKG_CONFIG_LIBDIR="${DUCK_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${DUCK_SYSROOT}/usr/share/pkgconfig"
  export PKG_CONFIG_ALLOW_CROSS=1
  export RUSTFLAGS="${RUSTFLAGS:+$RUSTFLAGS }-L ${DUCK_SYSROOT}/usr/lib/aarch64-linux-gnu"
fi
say "building ${DAEMONS[*]} in ${MICRODUCK_DIR} ($(git -C "$MICRODUCK_DIR" describe --always --dirty))"
(cd "$MICRODUCK_DIR" && cargo zigbuild --release --target "${TARGET}.2.31" "${pkgs[@]}")

STAGE="$(mktemp -d)"; trap 'rm -rf "$STAGE"' EXIT
mkdir -p "${STAGE}/bin" "${STAGE}/systemd/sysusers.d"
for d in "${DAEMONS[@]}"; do
  cp "${MICRODUCK_DIR}/target/${TARGET}/release/${d}" "${STAGE}/bin/"
  [[ "$d" != robotctl ]] || continue   # a client, no unit
  src="${MICRODUCK_DIR}/$(package_of "$d")/systemd"
  [[ -f "${src}/${d}.service" ]] || die "no ${src}/${d}.service"
  cp "${src}/${d}.service" "${STAGE}/systemd/"
  cp "${src}"/sysusers.d/*.conf "${STAGE}/systemd/sysusers.d/" 2>/dev/null || true
done
# No servo bus yet: run robotd against a robot made of nothing, holding the startup pose.
if [[ "$ROBOTD_FAKE" == "1" && " ${DAEMONS[*]} " == *" robotd "* ]]; then
  mkdir -p "${STAGE}/systemd/robotd.service.d"
  cat > "${STAGE}/systemd/robotd.service.d/fake.conf" <<'EOF'
# Installed by push-daemons.sh ROBOTD_FAKE=1. Delete this file and restart robotd to use the bus.
[Service]
ExecStart=
ExecStart=/opt/robot/daemon/current/bin/robotd --socket /run/robotd.sock --fake --no-policy
EOF
fi
# mediad on this board, always: the ISP main path by name (setup-media.sh makes the link), the
# full sensor array cropped 16:9 in the ISP because the RK3576 driver's 1080p mode is a 39° crop,
# and no software exposure loop because the rkaiq engine owns exposure (camera-setup.md).
if [[ " ${DAEMONS[*]} " == *" mediad "* ]]; then
  mkdir -p "${STAGE}/systemd/mediad.service.d"
  cat > "${STAGE}/systemd/mediad.service.d/rk3576.conf" <<EOF
# Installed by push-daemons.sh. The RK3576 camera path; see microduck-unitree docs/camera-setup.md.
[Service]
ExecStart=
ExecStart=/opt/robot/daemon/current/bin/mediad --camera-device /dev/camera-main --full-frame --no-auto-exposure --rotate ${CAMERA_ROTATE}
EOF
fi
# Every unit runs with SupplementaryGroups=robot, and upstream creates that group in updaterd's
# sysusers file, which is not installed here.
cp "${MICRODUCK_DIR}/updater/systemd/sysusers.d/robot.conf" "${STAGE}/systemd/sysusers.d/"
cp "${HERE}/install-daemons.sh" "${STAGE}/"
(cd "$STAGE" && find . -type f | sort | sed 's/^/   /')

if [[ "$DRY_RUN" == "1" ]]; then say "dry run: nothing copied"; exit 0; fi

REMOTE="/tmp/microduck-stage"
ssh "$BOARD" "rm -rf ${REMOTE} && mkdir -p ${REMOTE}"
scp -rq "${STAGE}/." "${BOARD}:${REMOTE}/"
ssh "$BOARD" "bash ${REMOTE}/install-daemons.sh ${REMOTE} ${OPERATOR} && rm -rf ${REMOTE}"
