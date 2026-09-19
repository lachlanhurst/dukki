#!/usr/bin/env bash
# Write Armbian to the Radxa CM4 eMMC over USB maskrom, with first-boot presets and an SSH key
# injected into the image so the board comes up on WiFi and is reachable without a console.
#
# Prerequisites: ./setup-flash-tools.sh has run; the module is on the CM4-NANO-A carrier with the
# BOOT switch ON and was powered up in that state; the carrier's USB-C is connected to this Mac.
#
#   WIFI_SSID='MyNet' WIFI_KEY='secret' USER_NAME='duck' USER_PASSWORD='pw' ROOT_PASSWORD='pw' ./flash-cm4.sh
#
# Optional environment:
#   RK_FLASH_DIR   where setup-flash-tools.sh put the tools (default $HOME/.microduck/rk-flash)
#   WIFI_COUNTRY   regulatory domain, default AU
#   SSH_PUBKEY     public key to authorise for root and the new user, default ~/.ssh/id_ed25519.pub
#   TIMEZONE       default Australia/Melbourne
#   LOCALE         default en_AU.UTF-8
#   DRY_RUN=1      prepare the image only, do not touch the board
set -euo pipefail

RK_FLASH_DIR="${RK_FLASH_DIR:-$HOME/.microduck/rk-flash}"
: "${WIFI_SSID:?set WIFI_SSID}" "${WIFI_KEY:?set WIFI_KEY}"
: "${USER_NAME:?set USER_NAME}" "${USER_PASSWORD:?set USER_PASSWORD}" "${ROOT_PASSWORD:?set ROOT_PASSWORD}"
WIFI_COUNTRY="${WIFI_COUNTRY:-AU}"
SSH_PUBKEY="${SSH_PUBKEY:-$HOME/.ssh/id_ed25519.pub}"
TIMEZONE="${TIMEZONE:-Australia/Melbourne}"
LOCALE="${LOCALE:-en_AU.UTF-8}"

cd "$RK_FLASH_DIR"
SRC=armbian.img
OUT=armbian-microduck.img
LOADER=rk3576_spl_loader.bin
DEBUGFS="$(brew --prefix e2fsprogs)/sbin/debugfs"
for f in "$SRC" "$LOADER" rkdeveloptool; do [[ -e $f ]] || { echo "missing $RK_FLASH_DIR/$f, run setup-flash-tools.sh"; exit 1; }; done
[[ -x $DEBUGFS ]] || { echo "missing debugfs (brew install e2fsprogs)"; exit 1; }

PUBKEY=""
if [[ -f $SSH_PUBKEY ]]; then PUBKEY="$(cat "$SSH_PUBKEY")"; else echo "note: no public key at $SSH_PUBKEY, password login only"; fi

# Locate the ext4 root partition inside the image from its GPT rather than assuming an offset.
ROOT_OFFSET=$(python3 - "$SRC" <<'PY'
import struct, sys
f = open(sys.argv[1], 'rb'); f.seek(512); h = f.read(92)
assert h[:8] == b'EFI PART', 'no GPT in image'
pe_lba, n, sz = struct.unpack('<QII', h[72:88]); f.seek(pe_lba * 512)
for i in range(n):
    e = f.read(sz)
    if e[:16] == b'\0' * 16: continue
    name = e[56:128].decode('utf-16le').rstrip('\0')
    if name == 'rootfs':
        print(struct.unpack('<Q', e[32:40])[0] * 512); break
else:
    sys.exit('rootfs partition not found')
PY
)
FS="$OUT?offset=$ROOT_OFFSET"
dfs() { $DEBUGFS -w -R "$1" "$FS" >/dev/null 2>&1; }

echo ">> preparing $OUT (rootfs at byte offset $ROOT_OFFSET)"
rm -f "$OUT"
cp -c "$SRC" "$OUT" 2>/dev/null || cp "$SRC" "$OUT"

# First-boot presets. Armbian sources this file with bash, so quote every value with %q.
# Do not use PRESET_ROOT_KEY / PRESET_USER_KEY: Armbian treats them as URLs to curl, not key text.
PRESET=$(mktemp)
q() { printf '%q' "$1"; }
cat > "$PRESET" <<PRESET_EOF
# Armbian first-boot presets, written by microduck-unitree scripts/compute/flash-cm4.sh
PRESET_NET_CHANGE_DEFAULTS=1
PRESET_NET_ETHERNET_ENABLED=0
PRESET_NET_WIFI_ENABLED=1
PRESET_NET_WIFI_SSID=$(q "$WIFI_SSID")
PRESET_NET_WIFI_KEY=$(q "$WIFI_KEY")
PRESET_NET_WIFI_COUNTRYCODE=$(q "$WIFI_COUNTRY")
PRESET_NET_USE_STATIC=0
SET_LANG_BASED_ON_LOCATION=n
PRESET_LOCALE=$(q "$LOCALE")
PRESET_TIMEZONE=$(q "$TIMEZONE")
PRESET_ROOT_PASSWORD=$(q "$ROOT_PASSWORD")
PRESET_USER_NAME=$(q "$USER_NAME")
PRESET_USER_PASSWORD=$(q "$USER_PASSWORD")
PRESET_DEFAULT_REALNAME=Microduck
PRESET_USER_SHELL=bash
PRESET_EOF
bash -n "$PRESET" || { echo "generated preset file does not parse"; exit 1; }

echo ">> injecting /root/.not_logged_in_yet"
dfs "rm /root/.not_logged_in_yet" || true
dfs "write $PRESET /root/.not_logged_in_yet"
dfs "sif /root/.not_logged_in_yet mode 0100600"
rm -f "$PRESET"
$DEBUGFS -R "cat /root/.not_logged_in_yet" "$FS" 2>/dev/null | sed -E 's/(WIFI_KEY|PASSWORD)=.*/\1=***/'

if [[ -n "$PUBKEY" ]]; then
  # /root/.ssh so root is reachable by key on first boot; /etc/skel/.ssh so useradd copies the
  # key into the new user's home when the first-login wizard creates the account.
  echo ">> injecting SSH public key for root and /etc/skel"
  KEYF=$(mktemp); printf '%s\n' "$PUBKEY" > "$KEYF"
  for d in /root/.ssh /etc/skel/.ssh; do
    dfs "mkdir $d" || true
    dfs "rm $d/authorized_keys" || true
    dfs "write $KEYF $d/authorized_keys"
    dfs "sif $d mode 040700"
    dfs "sif $d/authorized_keys mode 0100600"
  done
  rm -f "$KEYF"
fi

if [[ "${DRY_RUN:-0}" == "1" ]]; then echo ">> DRY_RUN set, image prepared, board untouched"; exit 0; fi

# Talk to the board. Note: with the RK3576 loader resident, rkdeveloptool still labels the device
# "Maskrom" and a repeat "db" fails, so test with rfi (read flash info) instead of trusting db.
echo ">> looking for the board"
./rkdeveloptool ld || { echo "no Rockchip USB device. BOOT switch on, then power, then USB-C to this Mac."; exit 1; }
if ! ./rkdeveloptool rfi >/dev/null 2>&1; then
  echo ">> uploading loader"
  ./rkdeveloptool db "$LOADER" || true
  sleep 3
  ./rkdeveloptool rfi >/dev/null 2>&1 || { echo "board not responding after loader upload; power cycle with BOOT on and retry"; exit 1; }
fi
./rkdeveloptool rfi | grep -m1 "Flash Size"

echo ">> writing $OUT to eMMC from LBA 0 (about 2.3 GB, several minutes)"
./rkdeveloptool wl 0 "$OUT"
echo ">> write complete, resetting board"
./rkdeveloptool rd || true
rm -f "$OUT"   # holds the WiFi key and passwords in clear text

cat <<MSG

Now: BOOT switch OFF, then power cycle. First boot takes a few minutes and joins "$WIFI_SSID".
Find the IP on your router, then log in as ROOT first:

    ssh root@<ip>

That first root login runs Armbian's wizard, which applies the presets and creates "$USER_NAME".
Wait for it to finish, then:  ssh $USER_NAME@<ip>
If the wizard fails, copy scripts/compute/board-finish-setup.sh to the board and run it as root.
MSG
