#!/usr/bin/env bash
# One-time creation of the Isaac Sim workstation on AWS.
#
# Creates (only what doesn't exist yet):
#   1. an EC2 key pair, private key saved to ~/.ssh/$KEY_NAME.pem
#   2. a security group allowing SSH from your current public IP only
#   3. the instance: Ubuntu 24.04, a root volume, and a separate data volume
#      that is kept when the instance is terminated
# The instance runs infra/bootstrap/user-data.sh on first boot.
#
# Usage: infra/scripts/provision.sh
# shellcheck source=lib.sh
source "$(dirname "$0")/lib.sh"
guard_account

[[ -z "$(instance_id)" ]] || die "an instance named '$NAME' already exists: $(instance_id)"

# --- 1. Key pair -------------------------------------------------------------
KEY_FILE="$HOME/.ssh/${KEY_NAME}.pem"
if aws ec2 describe-key-pairs --key-names "$KEY_NAME" >/dev/null 2>&1; then
  info "key pair $KEY_NAME exists"
  [[ -f "$KEY_FILE" ]] || die "key pair exists in AWS but $KEY_FILE is missing locally"
else
  info "creating key pair $KEY_NAME -> $KEY_FILE"
  mkdir -p "$HOME/.ssh"
  ( umask 077
    aws ec2 create-key-pair --key-name "$KEY_NAME" --key-type ed25519 \
      --query KeyMaterial --output text > "$KEY_FILE" )
fi

# --- 2. Security group -------------------------------------------------------
# Inbound: SSH from this machine's public IP only. No Isaac Sim ports:
# streaming goes over Tailscale, which needs no inbound rules.
VPC_ID="$(aws ec2 describe-vpcs --filters Name=is-default,Values=true \
          --query 'Vpcs[0].VpcId' --output text)"
[[ "$VPC_ID" != "None" ]] || die "no default VPC in $AWS_REGION"

SG_ID="$(aws ec2 describe-security-groups \
          --filters "Name=group-name,Values=$SG_NAME" "Name=vpc-id,Values=$VPC_ID" \
          --query 'SecurityGroups[0].GroupId' --output text)"
if [[ "$SG_ID" == "None" ]]; then
  info "creating security group $SG_NAME"
  SG_ID="$(aws ec2 create-security-group --group-name "$SG_NAME" --vpc-id "$VPC_ID" \
            --description "Isaac Sim workstation: SSH from owner IP only" \
            --query GroupId --output text)"
fi
MY_IP="$(curl -fsS https://checkip.amazonaws.com | tr -d '[:space:]')"
info "allowing SSH from $MY_IP/32 on $SG_ID"
aws ec2 authorize-security-group-ingress --group-id "$SG_ID" \
  --protocol tcp --port 22 --cidr "$MY_IP/32" >/dev/null 2>&1 \
  || info "(rule already present)"

# --- 3. Instance ---------------------------------------------------------------
# Latest Canonical Ubuntu 24.04 AMI, looked up via the public SSM parameter
# rather than a hard-coded AMI ID (AMI IDs differ per region and go stale).
AMI_ID="$(aws ssm get-parameter \
  --name /aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id \
  --query Parameter.Value --output text)"
info "AMI $AMI_ID"

# /dev/sda1: root.        Deleted on terminate (it only holds the OS + driver).
# /dev/sdf:  data volume. DeleteOnTermination=false so caches and the Docker
#            image survive even if the instance is rebuilt. On Nitro instances
#            these names show up as /dev/nvme*n1; bootstrap finds the disk by model.
BDM='[
  {"DeviceName":"/dev/sda1","Ebs":{"VolumeSize":'"$ROOT_GB"',"VolumeType":"gp3","DeleteOnTermination":true}},
  {"DeviceName":"/dev/sdf", "Ebs":{"VolumeSize":'"$DATA_GB"',"VolumeType":"gp3","DeleteOnTermination":false}}
]'

info "launching $INSTANCE_TYPE"
IID="$(aws ec2 run-instances \
  --image-id "$AMI_ID" \
  --instance-type "$INSTANCE_TYPE" \
  --key-name "$KEY_NAME" \
  --security-group-ids "$SG_ID" \
  --block-device-mappings "$BDM" \
  --instance-initiated-shutdown-behavior stop \
  --metadata-options HttpTokens=required,HttpEndpoint=enabled \
  --user-data "file://${INFRA_DIR}/bootstrap/user-data.sh" \
  --tag-specifications \
      "ResourceType=instance,Tags=[{Key=Name,Value=$NAME}]" \
      "ResourceType=volume,Tags=[{Key=Name,Value=$NAME}]" \
  --query 'Instances[0].InstanceId' --output text)"

info "waiting for $IID to run"
aws ec2 wait instance-running --instance-ids "$IID"
PUB_IP="$(aws ec2 describe-instances --instance-ids "$IID" \
          --query 'Reservations[0].Instances[0].PublicIpAddress' --output text)"

cat <<EOF

Instance $IID is running at $PUB_IP.
Bootstrap (driver, Docker, NVIDIA toolkit, Tailscale) runs on first boot and
reboots once; allow about 10 minutes. Watch it with:

  ssh -i $KEY_FILE ubuntu@$PUB_IP 'tail -f /var/log/isaac-bootstrap.log'

When the log ends with "BOOTSTRAP DONE", join your tailnet:

  ssh -i $KEY_FILE ubuntu@$PUB_IP 'sudo tailscale up --ssh'
EOF
