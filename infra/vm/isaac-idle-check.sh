#!/usr/bin/env bash
# Idle auto-shutdown for the Isaac Sim workstation.
# Installed as /usr/local/bin/isaac-idle-check and run every 5 minutes by
# isaac-idle.timer.
#
# The VM powers itself off once ALL of these have held for IDLE_MINUTES:
#   - no streaming client connected (no established TCP on SIGNAL_PORT)
#   - no SSH session (Tailscale SSH or regular sshd)
#   - GPU utilisation below GPU_BUSY_PCT (protects unattended batch jobs)
#   - no keep-awake file at /run/isaac-keepawake
# Because the instance's shutdown behaviour is "stop", a poweroff stops the
# EC2 instance: the GPU stops billing, the disks are kept.
#
# Logs go to the journal:  journalctl -t isaac-idle
set -euo pipefail

# Defaults; override them in /etc/default/isaac-idle.
IDLE_MINUTES=30      # how long everything must be idle before shutdown
GPU_BUSY_PCT=30      # average GPU use at or above this counts as busy
SIGNAL_PORT=49100    # Isaac Sim WebRTC signalling port
DRY_RUN=0            # 1 = log the decision but never power off
# shellcheck source=/dev/null
[[ -f /etc/default/isaac-idle ]] && source /etc/default/isaac-idle

# /run is a tmpfs, emptied on every boot, so a fresh boot always starts a
# full idle window instead of inheriting an old timestamp.
STATE=/run/isaac-idle-since
KEEPAWAKE=/run/isaac-keepawake

log() { logger -t isaac-idle "$*"; echo "$*"; }

# --- Collect reasons to stay awake --------------------------------------------
reasons=()

if [[ -e "$KEEPAWAKE" ]]; then
  reasons+=("keep-awake file")
fi

# Streaming client: the WebRTC client keeps a TCP connection open on the
# signalling port for the whole session. Isaac Sim also holds a permanent
# connection to its own signalling port from 127.0.0.1, so only connections
# from other machines (the peer address, column 4) count.
clients=$(ss -Htn state established "( sport = :${SIGNAL_PORT} )" \
  | awk '$4 !~ /^(127\.|\[::1\])/' | wc -l)
if (( clients > 0 )); then
  reasons+=("${clients} stream client(s)")
fi

# SSH: Tailscale SSH runs each session as a "tailscaled be-child ssh" process;
# regular sshd sessions are established TCP connections on port 22.
ts_ssh=$(pgrep -fc 'tailscaled be-child ssh' || true)
sshd=$(ss -Htn state established '( sport = :22 )' | wc -l)
if (( ts_ssh + sshd > 0 )); then
  reasons+=("$(( ts_ssh + sshd )) ssh session(s)")
fi

# GPU: average five one-second samples so one spike or dip doesn't decide.
gpu=0
for _ in 1 2 3 4 5; do
  u=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits | head -1)
  gpu=$(( gpu + u ))
  sleep 1
done
gpu=$(( gpu / 5 ))
if (( gpu >= GPU_BUSY_PCT )); then
  reasons+=("GPU ${gpu}%")
fi

# --- Decide ---------------------------------------------------------------------
now=$(date +%s)

if (( ${#reasons[@]} > 0 )); then
  rm -f "$STATE"                                   # activity resets the clock
  log "active: ${reasons[*]}"
  exit 0
fi

[[ -f "$STATE" ]] || echo "$now" > "$STATE"        # first idle check: start clock
idle_min=$(( (now - $(cat "$STATE")) / 60 ))

if (( idle_min < IDLE_MINUTES )); then
  log "idle ${idle_min}/${IDLE_MINUTES} min (GPU ${gpu}%)"
  exit 0
fi

log "idle ${idle_min} min >= ${IDLE_MINUTES}: shutting down"
if (( DRY_RUN )); then
  log "DRY_RUN=1, not shutting down"
  exit 0
fi

# Stop Isaac Sim first so its caches are fully written to /data, then power
# off. Shutdown behaviour "stop" turns this into an EC2 instance stop.
docker stop -t 30 isaac-sim >/dev/null 2>&1 || true
systemctl poweroff
