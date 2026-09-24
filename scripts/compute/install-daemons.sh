#!/usr/bin/env bash
# Runs ON the Radxa CM4 (Armbian, vendor kernel), as root. Installs a staged set of Microduck
# binaries, their systemd units and sysusers files in the layout upstream's install.sh produces,
# so the upstream unit files work unchanged and a later signed install simply replaces `current`.
# Normally run by push-daemons.sh from the Mac, which builds and stages the directory.
#
#   sudo bash install-daemons.sh <staged-dir> [operator]
#
# <staged-dir> holds bin/, and optionally systemd/*.service and systemd/sysusers.d/*.conf.
# [operator] is the login added to the `robot` group so robotctl reaches the sockets (default:
# duck). Safe to re-run: binaries and units are replaced, and only services whose unit was staged
# are restarted.
#
# Layout:
#   /opt/robot/daemon/hand/{bin,systemd}   this install ("hand": not signed, not from updaterd)
#   /opt/robot/daemon/current -> hand      what the unit files' ExecStart paths go through
#   /usr/local/bin/robotctl -> current/bin/robotctl
#   /etc/systemd/system/<unit>             copied, as install.sh does, not symlinked
set -euo pipefail

STAGE="${1:?usage: install-daemons.sh <staged-dir> [operator]}"
OPERATOR="${2:-duck}"
INSTALL_DIR=/opt/robot/daemon
RELEASE="${INSTALL_DIR}/hand"

say() { printf '>> %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "run as root"
[[ -d "${STAGE}/bin" ]] || die "no ${STAGE}/bin"

# 1. Refuse to overwrite a release the updater installed: that one is signed and journaled, and
#    swapping `current` under it would leave updaterd's record describing something else.
if [[ -L "${INSTALL_DIR}/current" && "$(readlink "${INSTALL_DIR}/current")" != "hand" ]]; then
  die "${INSTALL_DIR}/current points at $(readlink "${INSTALL_DIR}/current"), not a hand install"
fi

# 2. Binaries, replaced one by one so a partial stage (say, only tofd) keeps the others.
install -d "${RELEASE}/bin" "${RELEASE}/systemd/sysusers.d"
for b in "${STAGE}"/bin/*; do
  install -m 0755 "$b" "${RELEASE}/bin/$(basename "$b")"
  say "installed bin/$(basename "$b")"
done
ln -sfn hand "${INSTALL_DIR}/current"

# 3. Accounts. sysusers files first, because the units name users that must exist before start.
shopt -s nullglob
for f in "${STAGE}"/systemd/sysusers.d/*.conf; do
  install -m 0644 "$f" "${RELEASE}/systemd/sysusers.d/$(basename "$f")"
  install -m 0644 "$f" "/usr/lib/sysusers.d/$(basename "$f")"
  say "installed sysusers.d/$(basename "$f")"
done
systemd-sysusers
if id "$OPERATOR" >/dev/null 2>&1 && getent group robot >/dev/null; then
  if id -nG "$OPERATOR" | tr ' ' '\n' | grep -qx robot; then
    say "${OPERATOR} already in the robot group"
  else
    usermod -aG robot "$OPERATOR"
    say "added ${OPERATOR} to the robot group (takes effect at their next login)"
  fi
fi

# 4. robotctl on PATH through `current`, replacing a plain copy if one was put there by hand.
if [[ -x "${RELEASE}/bin/robotctl" ]]; then
  ln -sfn "${INSTALL_DIR}/current/bin/robotctl" /usr/local/bin/robotctl
  say "/usr/local/bin/robotctl -> current/bin/robotctl"
fi

# 5. Units: copy, enable, restart only what was staged.
units=()
for f in "${STAGE}"/systemd/*.service; do
  u="$(basename "$f")"
  install -m 0644 "$f" "${RELEASE}/systemd/${u}"
  install -m 0644 "$f" "/etc/systemd/system/${u}"
  units+=("$u")
  # Drop-ins staged with the unit (robotd's --fake override) replace whatever was there, so a
  # push without one removes a previous override rather than leaving it behind.
  rm -rf "/etc/systemd/system/${u}.d"
  if [[ -d "${STAGE}/systemd/${u}.d" ]]; then
    install -d "/etc/systemd/system/${u}.d"
    for c in "${STAGE}/systemd/${u}.d"/*.conf; do
      install -m 0644 "$c" "/etc/systemd/system/${u}.d/$(basename "$c")"
      say "installed ${u}.d/$(basename "$c")"
    done
  fi
done
if (( ${#units[@]} )); then
  systemctl daemon-reload
  for u in "${units[@]}"; do
    systemctl enable -q "$u"
    systemctl restart "$u"
    say "${u}: $(systemctl is-active "$u" || true)"
  done
fi

say "done"
