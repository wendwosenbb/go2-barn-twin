#!/usr/bin/env bash
# Copy the on-VM tools in infra/vm/ to the workstation and install them:
#   /usr/local/bin/isaac-run           start Isaac Sim headless
#   /usr/local/bin/isaac-idle-check    idle auto-shutdown check
#   /etc/default/isaac-idle            its settings (only if not present yet)
#   isaac-idle.service + .timer        run the check every 5 minutes
# Connects over Tailscale SSH, so the VM needs no open inbound ports.
#
# Usage: infra/scripts/push-vm-tools.sh [host]     (default host: isaac-sim-ws)
set -euo pipefail

HOST="${1:-isaac-sim-ws}"
VM_DIR="$(cd "$(dirname "$0")/../vm" && pwd)"

echo "==> copying $VM_DIR to $HOST"
ssh "$HOST" 'rm -rf /tmp/vm-tools && mkdir -p /tmp/vm-tools'
scp -q "$VM_DIR"/* "$HOST":/tmp/vm-tools/

echo "==> installing"
ssh "$HOST" 'sudo bash -s' <<'EOF'
set -euo pipefail
cd /tmp/vm-tools
install -m 0755 isaac-run.sh        /usr/local/bin/isaac-run
install -m 0755 isaac-idle-check.sh /usr/local/bin/isaac-idle-check
[ -f /etc/default/isaac-idle ] || install -m 0644 isaac-idle.conf /etc/default/isaac-idle
install -m 0644 isaac-idle.service isaac-idle.timer /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now isaac-idle.timer
systemctl list-timers isaac-idle.timer --no-pager
EOF
