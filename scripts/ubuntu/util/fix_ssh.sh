#!/usr/bin/env bash
set -euo pipefail

PI_USER="${PI_USER:-terralabscove}"
PI_PASS="${PI_PASS:-terralabscove}"
PI_LAN_HOST="${PI_LAN_HOST:-10.66.10.30}"
PI_TS_IP="${PI_TS_IP:-100.88.175.52}"
NM_IFACE="${NM_IFACE:-}"
INSTALL_DISPATCHER_HOOK="${INSTALL_DISPATCHER_HOOK:-1}"
DISABLE_PI_TAILSCALE_SSH="${DISABLE_PI_TAILSCALE_SSH:-1}"
STRICT_HOST_KEY_CHECKING="${STRICT_HOST_KEY_CHECKING:-no}"
HOOK_PATH="/etc/NetworkManager/dispatcher.d/90-restart-tailscale"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Applies the local Tailscale/OpenSSH recovery documented in README.md.

Options:
  --no-dispatcher-hook   Do not install the NetworkManager restart hook
  --no-pi-fix            Do not disable Tailscale SSH on the Pi
  -h, --help             Show this help

Environment overrides:
  PI_USER                Pi SSH username              (default: $PI_USER)
  PI_PASS                Pi SSH password              (default: terralabscove)
  PI_LAN_HOST            Pi LAN fallback host/IP       (default: $PI_LAN_HOST)
  PI_TS_IP               Pi Tailscale IP               (default: $PI_TS_IP)
  NM_IFACE               NetworkManager interface      (default: auto-detect)
  INSTALL_DISPATCHER_HOOK  1 to install hook, 0 to skip
  DISABLE_PI_TAILSCALE_SSH 1 to disable Pi Tailscale SSH, 0 to skip
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-dispatcher-hook)
      INSTALL_DISPATCHER_HOOK=0
      ;;
    --no-pi-fix)
      DISABLE_PI_TAILSCALE_SSH=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

log() {
  printf '\n==> %s\n' "$*"
}

warn() {
  printf 'warning: %s\n' "$*" >&2
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

require_cmd() {
  if ! have_cmd "$1"; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

run_sudo() {
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

detect_default_iface() {
  ip route show default 2>/dev/null | awk '{print $5; exit}'
}

show_tailscale_route() {
  ip -brief addr show tailscale0 || true
  ip route get "$PI_TS_IP" || true
}

restart_local_tailscale() {
  log "Restarting local Tailscale daemon"
  run_sudo systemctl enable --now tailscaled
  run_sudo systemctl restart tailscaled

  log "Bringing Tailscale back up"
  run_sudo tailscale up --timeout 5s

  local local_user
  local_user="${SUDO_USER:-${USER:-}}"
  if [[ -n "$local_user" && "$local_user" != "root" ]]; then
    log "Allowing $local_user to operate Tailscale without sudo"
    run_sudo tailscale set --operator="$local_user"
  fi
}

disable_pi_tailscale_ssh() {
  if [[ "$DISABLE_PI_TAILSCALE_SSH" != "1" ]]; then
    log "Skipping Pi Tailscale SSH change"
    return
  fi

  if ! have_cmd sshpass; then
    warn "sshpass is not installed; skipping Pi Tailscale SSH disable step"
    return
  fi

  log "Disabling Tailscale SSH on Pi through LAN fallback ${PI_USER}@${PI_LAN_HOST}"
  printf '%s\n' "$PI_PASS" | sshpass -p "$PI_PASS" ssh \
    -o ConnectTimeout=10 \
    -o StrictHostKeyChecking="$STRICT_HOST_KEY_CHECKING" \
    "${PI_USER}@${PI_LAN_HOST}" \
    "sudo -S tailscale set --ssh=false"
}

install_dispatcher_hook() {
  if [[ "$INSTALL_DISPATCHER_HOOK" != "1" ]]; then
    log "Skipping NetworkManager dispatcher hook"
    return
  fi

  local iface
  iface="$NM_IFACE"
  if [[ -z "$iface" ]]; then
    iface="$(detect_default_iface || true)"
  fi

  if [[ -z "$iface" ]]; then
    warn "could not auto-detect the default network interface; skipping dispatcher hook"
    warn "set NM_IFACE=enp0s5, or your current primary interface, and run again"
    return
  fi

  log "Installing NetworkManager dispatcher hook for interface $iface"
  local tmp
  tmp="$(mktemp)"
  cat >"$tmp" <<EOF
#!/usr/bin/env bash
set -euo pipefail

IFACE="\$1"
STATE="\$2"

case "\$STATE" in
  up|connectivity-change)
    if [[ "\$IFACE" == "$iface" ]]; then
      systemctl restart tailscaled
    fi
    ;;
esac
EOF

  run_sudo install -m 0755 -o root -g root "$tmp" "$HOOK_PATH"
  rm -f "$tmp"
}

verify_ssh() {
  if ! have_cmd sshpass; then
    warn "sshpass is not installed; skipping final SSH verification"
    return
  fi

  log "Verifying SSH over Tailscale at ${PI_USER}@${PI_TS_IP}"
  sshpass -p "$PI_PASS" ssh \
    -o ConnectTimeout=10 \
    -o StrictHostKeyChecking="$STRICT_HOST_KEY_CHECKING" \
    "${PI_USER}@${PI_TS_IP}" true
}

require_cmd ip
require_cmd awk
require_cmd systemctl
require_cmd sudo
require_cmd tailscale

log "Current Tailscale route state"
show_tailscale_route

restart_local_tailscale

log "Route state after local Tailscale recovery"
show_tailscale_route

disable_pi_tailscale_ssh
install_dispatcher_hook
verify_ssh

log "SSH fix complete"
