#!/usr/bin/env bash
# Start Isaac Sim headless, streaming over WebRTC on the Tailscale interface.
# Installed as /usr/local/bin/isaac-run. Extra arguments go straight to Kit,
# e.g.  isaac-run --/log/channels/omni.kit.livestream.streamsdk=info
set -euo pipefail

IMAGE=nvcr.io/nvidia/isaac-sim:6.1.0
CACHE=/data/isaac-sim

docker rm -f isaac-sim >/dev/null 2>&1 || true      # replace any old container
docker run -d --name isaac-sim --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  -e ISAACSIM_HOST="$(tailscale ip -4)" \
  -v "$CACHE/cache/main:/isaac-sim/.cache:rw" \
  -v "$CACHE/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$CACHE/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$CACHE/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$CACHE/hub:/var/cache/hub:rw" \
  -u 1234:1234 \
  --entrypoint bash "$IMAGE" -c "./runheadless.sh -v $*"
