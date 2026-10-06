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
