#!/usr/bin/env bash
# Show the state of the Isaac Sim workstation: instance, tailnet, Isaac Sim,
# and the idle auto-shutdown.
#
# Usage: infra/scripts/status.sh
# shellcheck source=lib.sh
source "$(dirname "$0")/lib.sh"

guard_account
IID="$(require_instance)"

read -r state type az < <(aws ec2 describe-instances --instance-ids "$IID" \
  --query 'Reservations[0].Instances[0].[State.Name,InstanceType,Placement.AvailabilityZone]' \
  --output text)
echo "instance:   $IID  $state  $type  $az"

if [[ "$state" != running ]]; then
  exit 0
fi

if ! tailscale ping -c 1 --timeout 5s "$VM_HOST" >/dev/null 2>&1; then
  echo "tailnet:    $VM_HOST not reachable (still booting, or this laptop is on another tailnet)"
  exit 0
fi
echo "tailnet:    $VM_HOST reachable"

vm 'bash -s' <<'EOF'
sim=$(docker ps --filter name='^isaac-sim$' --format '{{.Status}}')
echo "isaac sim:  ${sim:-not running}"
. /etc/default/isaac-idle 2>/dev/null
echo "auto-off:   after ${IDLE_MINUTES:-?} idle min (DRY_RUN=${DRY_RUN:-?})"
last=$(journalctl -t isaac-idle -n 1 --no-pager -o cat 2>/dev/null)
echo "last check: ${last:-none yet}"
echo "disk /data: $(df -h /data | awk 'NR==2 {print $3 " used of " $2}')"
EOF
