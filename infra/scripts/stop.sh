#!/usr/bin/env bash
# Stop the Isaac Sim workstation cleanly.
#
#   1. Stop the Isaac Sim container first. That flushes its caches to /data
#      and keeps the OS shutdown fast (with it running, a stop took ~7 min).
#   2. Stop the EC2 instance and wait until it is stopped.
#
# The disks are kept, so caches and images survive; they still bill
# (~$17/month for 210 GB gp3), the GPU does not.
#
# Usage: infra/scripts/stop.sh [--force]
#   --force   skip the clean shutdown inside the VM (like pulling the plug);
#             only for an instance stuck in "stopping"
# shellcheck source=lib.sh
source "$(dirname "$0")/lib.sh"

FORCE=0
if [[ "${1:-}" == "--force" ]]; then FORCE=1; fi

guard_account
IID="$(require_instance)"
state="$(instance_field "$IID" State.Name)"

if [[ "$state" == stopped ]]; then
  info "instance $IID is already stopped"
  exit 0
fi

# --- 1. Stop Isaac Sim inside the VM -------------------------------------------------
if [[ "$state" == running ]] && (( ! FORCE )); then
  info "stopping Isaac Sim on $VM_HOST"
  if timeout 90 ssh -o ConnectTimeout=10 "ubuntu@${VM_HOST}" \
       'docker stop -t 30 isaac-sim >/dev/null 2>&1 || true'; then
    info "  Isaac Sim stopped (or wasn't running)"
  else
    info "  could not reach $VM_HOST; stopping the instance anyway"
  fi
fi

# --- 2. Stop the instance --------------------------------------------------------------
stop_args=(--instance-ids "$IID")
if (( FORCE )); then stop_args+=(--force); fi
info "stopping instance $IID$( (( FORCE )) && echo ' (forced)')"
aws ec2 stop-instances "${stop_args[@]}" >/dev/null

info "waiting for it to stop"
aws ec2 wait instance-stopped --instance-ids "$IID" \
  || die "still not stopped after ~10 min; run: infra/scripts/stop.sh --force"
info "stopped. The GPU no longer bills; the disks (and caches) are kept."
