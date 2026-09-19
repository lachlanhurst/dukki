#!/usr/bin/env bash
# Runs ON the Radxa CM4 as root. Completes first-boot setup deterministically when Armbian's
# first-login wizard has failed or been skipped. Safe to re-run.
#
# Reads the presets left in /root/.not_logged_in_yet by flash-cm4.sh. Any of them can be
# overridden from the environment, and all are needed if that file is already gone:
#   PRESET_USER_NAME PRESET_USER_PASSWORD PRESET_ROOT_PASSWORD PRESET_TIMEZONE
#   PRESET_NET_WIFI_SSID PRESET_NET_WIFI_KEY PRESET_NET_WIFI_COUNTRYCODE
# Optional: DISABLE_PASSWORD_AUTH=1 turns off SSH password logins once a key is present for the user.
#
#   scp scripts/compute/board-finish-setup.sh root@<ip>:
#   ssh root@<ip> bash board-finish-setup.sh
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run as root"; exit 1; }

PRESET_FILE=/root/.not_logged_in_yet
if [[ -f $PRESET_FILE ]]; then
  # Environment wins over the file.
  env_user="${PRESET_USER_NAME:-}"; env_upw="${PRESET_USER_PASSWORD:-}"; env_rpw="${PRESET_ROOT_PASSWORD:-}"
  env_tz="${PRESET_TIMEZONE:-}"; env_ssid="${PRESET_NET_WIFI_SSID:-}"; env_key="${PRESET_NET_WIFI_KEY:-}"; env_cc="${PRESET_NET_WIFI_COUNTRYCODE:-}"
  # shellcheck disable=SC1090
  source "$PRESET_FILE"
  PRESET_USER_NAME="${env_user:-${PRESET_USER_NAME:-}}"; PRESET_USER_PASSWORD="${env_upw:-${PRESET_USER_PASSWORD:-}}"
  PRESET_ROOT_PASSWORD="${env_rpw:-${PRESET_ROOT_PASSWORD:-}}"; PRESET_TIMEZONE="${env_tz:-${PRESET_TIMEZONE:-}}"
  PRESET_NET_WIFI_SSID="${env_ssid:-${PRESET_NET_WIFI_SSID:-}}"; PRESET_NET_WIFI_KEY="${env_key:-${PRESET_NET_WIFI_KEY:-}}"
  PRESET_NET_WIFI_COUNTRYCODE="${env_cc:-${PRESET_NET_WIFI_COUNTRYCODE:-AU}}"
fi
: "${PRESET_USER_NAME:?no user name}" "${PRESET_USER_PASSWORD:?no user password}" "${PRESET_ROOT_PASSWORD:?no root password}"
PRESET_TIMEZONE="${PRESET_TIMEZONE:-Australia/Melbourne}"

# 1. WiFi: make sure the netplan file carries the passphrase. A wizard run that failed part way
#    can leave this file without one, which breaks WiFi on the next boot.
NP=/etc/netplan/30-wifis-dhcp.yaml
if [[ -n "${PRESET_NET_WIFI_SSID:-}" && -n "${PRESET_NET_WIFI_KEY:-}" ]]; then
  WLAN="$(iw dev 2>/dev/null | awk '$1=="Interface"{print $2}' | head -n1)"
  WLAN="${WLAN:-wlan0}"
  cat > "$NP.tmp" <<YAML
network:
  wifis:
    $WLAN:
      dhcp4: yes
      dhcp6: yes
      regulatory-domain: $PRESET_NET_WIFI_COUNTRYCODE
      access-points:
        "$PRESET_NET_WIFI_SSID":
          password: "$PRESET_NET_WIFI_KEY"
YAML
  chmod 600 "$NP.tmp"; mv -f "$NP.tmp" "$NP"
  netplan generate && echo ">> netplan: $NP written and valid (applies on next boot)"
else
  grep -q "password:" "$NP" 2>/dev/null && echo ">> netplan: existing WiFi config has a passphrase" \
    || echo "WARNING: $NP has no WiFi passphrase and none was given. Fix before rebooting."
fi

# 2. Accounts.
echo "root:$PRESET_ROOT_PASSWORD" | chpasswd && echo ">> root password set"
if ! id "$PRESET_USER_NAME" >/dev/null 2>&1; then
  useradd --create-home --shell /bin/bash --comment "Microduck" "$PRESET_USER_NAME"
  echo ">> user $PRESET_USER_NAME created (home from /etc/skel, including any injected SSH key)"
fi
echo "$PRESET_USER_NAME:$PRESET_USER_PASSWORD" | chpasswd && echo ">> user password set"
# Same group list Armbian's wizard uses. dialout is the one the servo bridge UART needs.
for g in sudo netdev audio video disk tty users games dialout plugdev input bluetooth systemd-journal ssh render; do
  getent group "$g" >/dev/null && usermod -aG "$g" "$PRESET_USER_NAME"
done
echo ">> groups: $(id -nG "$PRESET_USER_NAME")"

# 3. Time zone.
timedatectl set-timezone "$PRESET_TIMEZONE" && echo ">> timezone $PRESET_TIMEZONE"

# 4. Stop the wizard from re-running on root logins; it holds passwords in clear text.
rm -f "$PRESET_FILE"

# 5. Optional hardening.
AK="/home/$PRESET_USER_NAME/.ssh/authorized_keys"
if [[ "${DISABLE_PASSWORD_AUTH:-0}" == "1" ]]; then
  if [[ -s $AK ]]; then
    printf 'PasswordAuthentication no\nKbdInteractiveAuthentication no\n' > /etc/ssh/sshd_config.d/50-keys-only.conf
    systemctl restart ssh && echo ">> SSH password logins disabled"
  else
    echo "not disabling password auth: $AK is missing or empty"
  fi
fi

echo ">> done. Log in with: ssh $PRESET_USER_NAME@$(hostname -I 2>/dev/null | awk '{print $1}')"
