# Running Isaac Sim on a cloud GPU

Infrastructure for running NVIDIA Isaac Sim 6.1 headless on an AWS GPU
instance, viewed from a laptop through the WebRTC streaming client over a
private Tailscale network.

> Work in progress (Unit 0). Start/stop scripts, idle auto-shutdown and the
> full walkthrough are added in later lessons.

## Layout

| Path | Purpose |
|---|---|
| `config.env.example` | Settings template; copy to `config.env` (git-ignored) |
| `scripts/lib.sh` | Shared helpers, including the AWS account guard |
| `scripts/provision.sh` | One-time creation of key pair, security group and instance |
| `bootstrap/user-data.sh` | First-boot setup: data volume, NVIDIA driver, Docker, NVIDIA Container Toolkit, Tailscale |

## Design

- **Instance:** `g6.2xlarge` (NVIDIA L4 24 GB, 32 GB RAM), the smallest AWS
  instance meeting Isaac Sim's minimum spec. A100/H100 lack RT cores and
  cannot render Isaac Sim.
- **Two disks:** a disposable root volume for the OS and driver, and a data
  volume at `/data` holding Docker images and Isaac Sim caches. The data volume
  is kept when the instance is terminated, so warm shader caches survive a rebuild.
- **No public Isaac Sim ports.** WebRTC streaming has no authentication or
  encryption, so it runs over Tailscale. The security group allows SSH from
  the owner's IP only.
- **Account guard:** every script checks the AWS account ID before acting.
