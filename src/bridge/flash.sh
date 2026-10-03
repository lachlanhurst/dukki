#!/usr/bin/env bash
# Build the bridge firmware and flash it over USB DFU.
# Put the board in its ROM bootloader first: hold BOOT, tap reset, release BOOT.
set -euo pipefail
cd "$(dirname "$0")"
export PATH="/opt/homebrew/opt/rustup/bin:$HOME/.cargo/bin:$PATH"

cargo objcopy --release -- -O binary target/bridge.bin
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D target/bridge.bin
