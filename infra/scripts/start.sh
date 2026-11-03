#!/usr/bin/env bash
# Start the Isaac Sim workstation and launch Isaac Sim, ready for the client.
#
#   1. Start the EC2 instance as INSTANCE_TYPE. On InsufficientInstanceCapacity,
#      wait and retry the same type (no fallback to other types).
#   2. Wait until the VM answers on the tailnet.
#   3. Warn if idle auto-shutdown is still set to a test value.
#   4. Start Isaac Sim (isaac-run) and wait for "Full Streaming App is loaded".
#
# Usage: infra/scripts/start.sh [--vm-only]
#   --vm-only   start the VM but not Isaac Sim (e.g. for batch jobs)
# shellcheck source=lib.sh
source "$(dirname "$0")/lib.sh"

VM_ONLY=0
if [[ "${1:-}" == "--vm-only" ]]; then VM_ONLY=1; fi

guard_account
IID="$(require_instance)"
T0=$(date +%s)

# try_start TYPE: switch the (stopped) instance to TYPE and start it.
# Returns 0 on success, 2 when AWS has no capacity for TYPE in this zone.
# Any other error stops the script.
try_start() {
  local type="$1" out
  if [[ "$(instance_field "$IID" InstanceType)" != "$type" ]]; then
    aws ec2 modify-instance-attribute --instance-id "$IID" \
      --instance-type "{\"Value\": \"$type\"}" || die "could not switch to $type"
  fi
  if out="$(aws ec2 start-instances --instance-ids "$IID" 2>&1)"; then
    return 0
  fi
  if grep -q InsufficientInstanceCapacity <<<"$out"; then
    return 2
  fi
  die "start failed: $out"
}

# --- 1. Start the instance --------------------------------------------------------
state="$(instance_field "$IID" State.Name)"
if [[ "$state" == stopping ]]; then
  info "instance is still stopping; waiting for that to finish first"
  aws ec2 wait instance-stopped --instance-ids "$IID"
  state=stopped
fi

if [[ "$state" == stopped ]]; then
  AZ="$(instance_field "$IID" Placement.AvailabilityZone)"
  started=""
  for attempt in $(seq "$START_ROUNDS"); do
    info "starting $IID as $INSTANCE_TYPE (attempt $attempt/$START_ROUNDS)"
    if try_start "$INSTANCE_TYPE"; then
      started=1
      break
    fi
    if (( attempt < START_ROUNDS )); then
      info "  no $INSTANCE_TYPE capacity in $AZ; retrying in ${ROUND_WAIT}s"
      sleep "$ROUND_WAIT"
    fi
  done
  [[ -n "$started" ]] || die "no $INSTANCE_TYPE capacity in $AZ after $START_ROUNDS attempts; try again later"
else
  info "instance $IID is already $state"
fi

aws ec2 wait instance-running --instance-ids "$IID"
info "instance running ($(instance_field "$IID" InstanceType))"

# --- 2. Reach it over the tailnet --------------------------------------------------
wait_tailnet

# --- 3. Check the idle auto-shutdown settings --------------------------------------
# Single quotes on purpose: the variables expand on the VM, not here.
# shellcheck disable=SC2016
read -r idle_min dry_run < <(vm '. /etc/default/isaac-idle 2>/dev/null; echo "${IDLE_MINUTES:-?} ${DRY_RUN:-?}"')
info "idle auto-shutdown after ${idle_min} min (DRY_RUN=${dry_run})"
if [[ "$dry_run" != 0 ]] || { [[ "$idle_min" =~ ^[0-9]+$ ]] && (( idle_min < 15 )); }; then
  echo "WARNING: idle shutdown looks like a leftover test setting. Reset with:" >&2
  echo "  ssh ubuntu@$VM_HOST 'sudo sed -i -e \"s/^IDLE_MINUTES=.*/IDLE_MINUTES=30/\" -e \"s/^DRY_RUN=.*/DRY_RUN=0/\" /etc/default/isaac-idle'" >&2
fi

if (( VM_ONLY )); then
  info "VM ready in $(( $(date +%s) - T0 ))s (Isaac Sim not started). ssh ubuntu@$VM_HOST"
  exit 0
fi

# --- 4. Start Isaac Sim and wait until it is ready ---------------------------------
# Don't restart a running Isaac Sim: that would cut off a live session.
if vm "docker ps -q --filter name='^isaac-sim$' --filter status=running | grep -q ."; then
  info "Isaac Sim is already running (to restart it: ssh ubuntu@$VM_HOST isaac-run)"
else
  info "starting Isaac Sim"
  vm 'isaac-run >/dev/null'
fi
info "waiting for Isaac Sim to load (warm ~1 min, cold ~5 min)"

# Poll the log every 5 s (up to 15 min). Print Kit's own load time on success;
# exit 2 if the container dies, 1 on timeout.
load_s="$(vm 'bash -s' <<'EOF'
for i in $(seq 180); do
  line=$(docker logs isaac-sim 2>&1 | grep -m1 "Full Streaming App is loaded" || true)
  if [ -n "$line" ]; then
    echo "$line" | grep -o '\[[0-9.]*s\]' | tail -1 | tr -d '[]s'
    exit 0
  fi
  if ! docker ps -q --filter name='^isaac-sim$' --filter status=running | grep -q .; then
    exit 2
  fi
  sleep 5
done
exit 1
EOF
)" || die "Isaac Sim did not load. Look at: ssh ubuntu@$VM_HOST 'docker logs --tail 50 isaac-sim'"

TS_IP="$(vm 'tailscale ip -4')"
info "Isaac Sim loaded in ${load_s}s (total $(( $(date +%s) - T0 ))s from start.sh)"
info "connect the WebRTC streaming client to $TS_IP"
