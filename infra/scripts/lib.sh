# shellcheck shell=bash
# Shared helpers, sourced by every script in infra/scripts/.
# Not meant to be executed directly.

set -euo pipefail

INFRA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${INFRA_DIR}/config.env"

die()  { echo "ERROR: $*" >&2; exit 1; }
info() { echo "==> $*"; }

[[ -f "$CONFIG" ]] || die "missing $CONFIG (copy config.env.example and fill it in)"
# shellcheck source=/dev/null
source "$CONFIG"

# Defaults for settings added after the first config.env was written, so an
# older config.env keeps working.
: "${VM_HOST:=isaac-sim-ws}"                   # tailnet hostname and ssh target
: "${START_ROUNDS:=30}"                        # start attempts on no capacity (~30 min)
: "${ROUND_WAIT:=60}"                          # seconds between attempts

# Export so every `aws` call below uses the personal profile and region,
# whatever the calling terminal has set.
export AWS_PROFILE AWS_REGION
export AWS_DEFAULT_REGION="$AWS_REGION"

command -v aws >/dev/null || die "aws CLI not found"

# Account guard: refuse to touch anything unless these credentials belong
# to the expected personal account.
guard_account() {
  local actual
  actual="$(aws sts get-caller-identity --query Account --output text)" \
    || die "could not call STS with profile '$AWS_PROFILE'"
  [[ "$actual" == "$EXPECTED_ACCOUNT_ID" ]] \
    || die "profile '$AWS_PROFILE' is account $actual, expected $EXPECTED_ACCOUNT_ID. Refusing to continue."
  info "account $actual ($AWS_PROFILE, $AWS_REGION)"
}

# Instance ID of the workstation (any state except terminated), or empty.
instance_id() {
  aws ec2 describe-instances \
    --filters "Name=tag:Name,Values=$NAME" \
              "Name=instance-state-name,Values=pending,running,stopping,stopped" \
    --query 'Reservations[].Instances[].InstanceId' --output text
}

# Instance ID, or exit with a hint if there is none.
require_instance() {
  local iid
  iid="$(instance_id)"
  [[ -n "$iid" ]] || die "no instance named '$NAME' (run provision.sh first)"
  echo "$iid"
}

# One field of the instance, e.g. instance_field "$IID" State.Name
instance_field() {
  aws ec2 describe-instances --instance-ids "$1" \
    --query "Reservations[0].Instances[0].$2" --output text
}

# Run a command on the VM over Tailscale SSH.
vm() {
  ssh -o ConnectTimeout=10 "ubuntu@${VM_HOST}" "$@"
}

# Wait until the VM answers on the tailnet (up to ~3 minutes after boot).
wait_tailnet() {
  local _
  info "waiting for $VM_HOST on the tailnet"
  for _ in $(seq 36); do
    if tailscale ping -c 1 --timeout 5s "$VM_HOST" >/dev/null 2>&1; then
      return 0
    fi
    sleep 5
  done
  die "$VM_HOST not reachable on the tailnet. Is this laptop on the right tailnet? (sudo tailscale switch <your-tailnet>)"
}
