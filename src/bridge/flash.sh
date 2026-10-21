#!/usr/bin/env bash
# Build the bridge firmware and flash it over USB DFU.
# With the bridge's USB console present, its `U` command restarts it into the ROM bootloader.
# Without it (no console, or firmware from before `U`), put the board in its bootloader by hand:
# hold BOOT, tap reset, release BOOT.
set -euo pipefail
cd "$(dirname "$0")"
export PATH="/opt/homebrew/opt/rustup/bin:$HOME/.cargo/bin:$PATH"

cargo objcopy --release -- -O binary target/bridge.bin

dfu_present() { dfu-util -l 2>/dev/null | grep -q "0483:df11"; }
if ! dfu_present; then
  console=$(ls /dev/cu.usbmodembridge* 2>/dev/null | head -1 || true)
  [[ -n "$console" ]] || { echo "no bridge console and no DFU device: hold BOOT, tap reset" >&2; exit 1; }
  echo "restarting the bridge into its bootloader through ${console}"
  printf U > "$console"
  for _ in $(seq 50); do dfu_present && break; sleep 0.1; done
  dfu_present || { echo "no DFU device after the console's U: hold BOOT, tap reset" >&2; exit 1; }
fi
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D target/bridge.bin
