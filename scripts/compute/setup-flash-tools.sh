#!/usr/bin/env bash
# Fetch and build everything needed to flash the Radxa CM4 (RK3576) eMMC from a Mac.
# Idempotent: re-running skips anything already present and verified.
#
#   ./setup-flash-tools.sh              # tools into $HOME/.microduck/rk-flash
#   RK_FLASH_DIR=/some/dir ./setup-flash-tools.sh
#
# Produces in RK_FLASH_DIR:
#   rkdeveloptool                    Rockchip USB flashing tool, built from source with clang
#   rk3576_spl_loader.bin            RK3576 SPL loader from Radxa's rkbin (pinned)
#   armbian.img.xz / armbian.img     Armbian for radxa-cm4-io (pinned release, sha256 checked)
# Requires: Homebrew (for e2fsprogs and libusb), Xcode command line tools (clang, git), python3, xz.
set -euo pipefail

RK_FLASH_DIR="${RK_FLASH_DIR:-$HOME/.microduck/rk-flash}"

# Pinned inputs. Update these together and re-verify the whole procedure.
RKDEVELOPTOOL_REPO="https://github.com/rockchip-linux/rkdeveloptool.git"
RKDEVELOPTOOL_COMMIT="304f073752fd25c854e1bcf05d8e7f925b1f4e14"   # 07/03/2025, v1.32 era
LOADER_URL="https://raw.githubusercontent.com/radxa/rkbin/develop-v2026.01/bin/rk35/rk3576_spl_loader_v1.12.108.bin"
LOADER_SHA256="01814239f4fad8adfd93aae429c0ccae78e999f35637f8ef27b9dfc64e561028"
IMAGE_URL="https://dl.armbian.com/radxa-cm4-io/archive/Armbian_26.8.2_Radxa-cm4-io_trixie_vendor_6.1.115_minimal.img.xz"
IMAGE_SHA256="511e797e6b5fb3a072609495cd235338895f723509bd116b76fea213c4f06179"

mkdir -p "$RK_FLASH_DIR"
cd "$RK_FLASH_DIR"
echo ">> work dir: $RK_FLASH_DIR"

sha_ok() { [[ -f "$1" ]] && [[ "$(shasum -a 256 "$1" | cut -d' ' -f1)" == "$2" ]]; }

# 1. Homebrew packages: libusb for rkdeveloptool, e2fsprogs for debugfs (edits ext4 inside the image).
command -v brew >/dev/null || { echo "Homebrew is required: https://brew.sh"; exit 1; }
for pkg in libusb e2fsprogs; do
  brew list --versions "$pkg" >/dev/null 2>&1 || { echo ">> brew install $pkg"; brew install "$pkg"; }
done
DEBUGFS="$(brew --prefix e2fsprogs)/sbin/debugfs"
[[ -x $DEBUGFS ]] || { echo "debugfs not found at $DEBUGFS"; exit 1; }

# 2. rkdeveloptool. Homebrew has no formula and the upstream autotools build does not work with
#    the autotools commonly found on a Mac, so compile the sources directly with clang.
if [[ ! -x rkdeveloptool ]]; then
  echo ">> building rkdeveloptool at $RKDEVELOPTOOL_COMMIT"
  rm -rf rkdeveloptool-src
  git clone -q "$RKDEVELOPTOOL_REPO" rkdeveloptool-src
  git -C rkdeveloptool-src checkout -q "$RKDEVELOPTOOL_COMMIT"
  ( cd rkdeveloptool-src
    printf '#define PACKAGE_VERSION "1.32-%s"\n' "${RKDEVELOPTOOL_COMMIT:0:7}" > config.h
    clang++ -std=c++11 -O2 -Wno-everything -fno-strict-aliasing -D_FILE_OFFSET_BITS=64 -D_LARGE_FILE \
      $(pkg-config --cflags libusb-1.0) -I. -o ../rkdeveloptool ./*.cpp $(pkg-config --libs libusb-1.0) )
  rm -rf rkdeveloptool-src
fi
./rkdeveloptool -v

# 3. RK3576 SPL loader.
if ! sha_ok rk3576_spl_loader.bin "$LOADER_SHA256"; then
  echo ">> downloading loader"
  curl -fsSL -o rk3576_spl_loader.bin "$LOADER_URL"
  sha_ok rk3576_spl_loader.bin "$LOADER_SHA256" || { echo "loader checksum mismatch"; exit 1; }
fi
echo ">> loader ok"

# 4. Armbian image.
if ! sha_ok armbian.img.xz "$IMAGE_SHA256"; then
  echo ">> downloading Armbian image (about 400 MB)"
  curl -fL -o armbian.img.xz "$IMAGE_URL"
  sha_ok armbian.img.xz "$IMAGE_SHA256" || { echo "image checksum mismatch"; exit 1; }
fi
if [[ ! -f armbian.img ]]; then
  echo ">> decompressing image"
  xz -dk -T0 armbian.img.xz
fi
echo ">> image ok: $(basename "$IMAGE_URL")"

echo
echo "Ready. Next: put the board in maskrom mode (BOOT switch on, then power) and run flash-cm4.sh"
