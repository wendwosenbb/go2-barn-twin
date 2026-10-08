#!/usr/bin/env bash
# First-boot bootstrap for the Isaac Sim workstation (Ubuntu 24.04, EC2 g6).
# Passed to EC2 as user-data; cloud-init runs it once as root.
#
# Installs and configures, in order:
#   1. the data volume, mounted at /data
#   2. the NVIDIA driver (595 production branch)
#   3. Docker, storing images on /data/docker
#   4. the NVIDIA Container Toolkit
#   5. Tailscale (joined manually afterwards, so no auth key lives in user-data)
#   6. Isaac Sim cache directories, owned by the container user UID 1234
# then reboots once to load the driver.
#
# Idempotent: safe to re-run with `sudo bash user-data.sh`.
set -euo pipefail
exec > >(tee -a /var/log/isaac-bootstrap.log) 2>&1
echo "=== bootstrap start $(date -Is)"

export DEBIAN_FRONTEND=noninteractive
DRIVER_BRANCH=595        # Isaac Sim 6.1 was tested on 595.58.03

# --- 1. Data volume ------------------------------------------------------------
# g6 instances have TWO kinds of NVMe disk: EBS volumes (persistent) and local
# instance store (wiped on every stop). Pick by model name, never by device
# number, which can change between boots.
ROOT_DISK="$(lsblk -no PKNAME "$(findmnt -no SOURCE /)")"
DATA_DISK=""
for d in $(lsblk -dno NAME); do
  model="$(lsblk -dno MODEL "/dev/$d" | xargs)"
  if [[ "$model" == "Amazon Elastic Block Store" && "$d" != "$ROOT_DISK" ]]; then
    DATA_DISK="/dev/$d"
  fi
done
[[ -n "$DATA_DISK" ]] || { echo "no EBS data disk found"; lsblk; exit 1; }
echo "data disk: $DATA_DISK"

# Format only if it has no filesystem, so a re-attached volume keeps its data.
if ! blkid "$DATA_DISK" >/dev/null 2>&1; then
  mkfs.ext4 -L isaacdata "$DATA_DISK"
fi
mkdir -p /data
UUID="$(blkid -s UUID -o value "$DATA_DISK")"
grep -q "$UUID" /etc/fstab || \
  echo "UUID=$UUID /data ext4 defaults,nofail,x-systemd.device-timeout=10 0 2" >> /etc/fstab
mountpoint -q /data || mount /data

# --- 2. NVIDIA driver ----------------------------------------------------------
apt-get update
apt-get install -y "linux-headers-$(uname -r)" build-essential curl acl jq
# The -server packages are NVIDIA's long-term data-center flavour and include
# the graphics/Vulkan libraries Isaac Sim's renderer needs.
if ! dpkg -l | grep -q "nvidia-driver-${DRIVER_BRANCH}"; then
  apt-get install -y "nvidia-driver-${DRIVER_BRANCH}-server" \
    || apt-get install -y "nvidia-driver-${DRIVER_BRANCH}"
fi

# --- 3. Docker, with its data root on /data ------------------------------------
# Written before Docker first starts, so the multi-GB Isaac Sim image is stored
# on the persistent data volume rather than the root disk.
mkdir -p /etc/docker /data/docker
[[ -f /etc/docker/daemon.json ]] || echo '{ "data-root": "/data/docker" }' > /etc/docker/daemon.json
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sh
fi
usermod -aG docker ubuntu

# --- 4. NVIDIA Container Toolkit -----------------------------------------------
if ! command -v nvidia-ctk >/dev/null; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
fi
nvidia-ctk runtime configure --runtime=docker   # merges into daemon.json

# --- 4b. Keep image layers on /data too ------------------------------------------
# Recent Docker Engine stores image layers through containerd's image store,
# under containerd's own root (/var/lib/containerd), NOT Docker's data-root.
# Point containerd's root at the data volume as well.
mkdir -p /data/containerd
grep -q '^root *= *"/data/containerd"' /etc/containerd/config.toml 2>/dev/null \
  || sed -i '1i root = "/data/containerd"' /etc/containerd/config.toml
# Neither daemon may start before the data volume is mounted.
for svc in containerd docker; do
  mkdir -p "/etc/systemd/system/${svc}.service.d"
  printf '[Unit]\nRequiresMountsFor=/data\n' > "/etc/systemd/system/${svc}.service.d/data-mount.conf"
done
systemctl daemon-reload
systemctl restart containerd docker

# --- 5. Tailscale --------------------------------------------------------------
command -v tailscale >/dev/null || curl -fsSL https://tailscale.com/install.sh | sh

# --- 6. Isaac Sim cache directories --------------------------------------------
# The container runs as UID 1234; its mount points must be writable by it.
for d in cache/main cache/computecache cache/kit config data logs pkg hub; do
  mkdir -p "/data/isaac-sim/$d"
done
chown -R 1234:1234 /data/isaac-sim

# --- Reboot once to load the driver --------------------------------------------
if [[ ! -f /var/lib/isaac-bootstrap.done ]]; then
  touch /var/lib/isaac-bootstrap.done
  echo "=== BOOTSTRAP DONE $(date -Is) (rebooting to load the NVIDIA driver)"
  # `reboot` only asks systemd to reboot and returns at once; exit so the
  # script doesn't run on and log a second "BOOTSTRAP DONE".
  reboot
  exit 0
fi
echo "=== BOOTSTRAP DONE $(date -Is)"
