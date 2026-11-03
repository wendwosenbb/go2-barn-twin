# Running Isaac Sim on a cloud GPU

Infrastructure for running NVIDIA Isaac Sim 6.1 headless on an AWS GPU
instance, viewed from a laptop through the WebRTC streaming client over a
private Tailscale network, with:

- **one command to start and one to stop** (`start.sh`, `stop.sh`)
- **persistent shader and asset caches**, so a warm start takes ~50 s instead of ~5 min
- **no open inbound ports**: all access goes over a private Tailscale network
- **idle auto-shutdown**, so a forgotten instance stops itself after 30 minutes
- **capacity retry**, so a temporary "no GPUs available" doesn't need babysitting

Everything here was built and measured on a `g6.2xlarge` (NVIDIA L4) in
`us-east-2`.

## Layout

| Path | Purpose |
|---|---|
| `config.env.example` | Settings template; copy to `config.env` (git-ignored) |
| `scripts/lib.sh` | Shared helpers, including the AWS account guard |
| `scripts/provision.sh` | One-time creation of key pair, security group and instance |
| `scripts/push-vm-tools.sh` | Install `vm/*` on the VM over Tailscale SSH |
| `scripts/start.sh` | Start the VM and Isaac Sim, retrying while AWS has no capacity |
| `scripts/stop.sh` | Stop Isaac Sim, then the instance |
| `scripts/status.sh` | Instance, tailnet, Isaac Sim and auto-shutdown state at a glance |
| `bootstrap/user-data.sh` | First-boot setup: data volume, NVIDIA driver, Docker, NVIDIA Container Toolkit, containerd on `/data`, Tailscale, cache dirs |
| `vm/isaac-run.sh` | Start Isaac Sim headless (installed as `isaac-run`) |
| `vm/isaac-idle-*` | Idle detector, its settings, and the systemd service and timer |

## Architecture

```
 laptop                                     AWS us-east-2 (default VPC)
 ┌────────────────────────┐                 ┌───────────────────────────────────────────┐
 │ WebRTC streaming client│                 │ g6.2xlarge  (NVIDIA L4, 32 GB RAM)          │
 │ start.sh / stop.sh     │   Tailscale     │                                             │
 │ aws CLI (own profile)  │◄═══════════════►│  tailscaled ── Tailscale SSH                │
 └────────────────────────┘   WireGuard     │  docker --network=host                      │
                              (UDP, NAT     │   └─ isaac-sim:6.1.0  runheadless.sh        │
                               traversal)   │        signalling TCP 49100, media UDP 47998│
                                            │        bound to the Tailscale IP            │
                                            │  isaac-idle.timer  (every 5 min)            │
                                            │                                             │
                                            │  root EBS 60 GB   OS + NVIDIA driver 595    │
                                            │  data EBS 150 GB  /data: container images,  │
                                            │                   Isaac Sim caches          │
                                            │                   (kept if instance dies)   │
                                            └───────────────────────────────────────────┘
                                             security group: no inbound rules at all
```

## Design

- **Instance:** `g6.2xlarge` (NVIDIA L4 24 GB, 32 GB RAM), the smallest AWS
  instance meeting Isaac Sim's minimum spec. A100/H100 lack RT cores and
  cannot render Isaac Sim.
- **Two disks:** a disposable root volume for the OS and driver, and a data
  volume at `/data` holding Docker images and Isaac Sim caches. The data volume
  is kept when the instance is terminated, so warm shader caches survive a rebuild.
- **No public Isaac Sim ports.** WebRTC streaming has no authentication or
  encryption, so it runs over Tailscale. The WebRTC media socket must also bind
  to an address on an interface inside the container: a cloud VM's public IP
  isn't (it's NAT'd), but its Tailscale IP is. The security group allowed SSH
  from the owner's IP only during setup; after Tailscale SSH is up it has no
  inbound rules at all.
- **Account guard:** every script checks the AWS account ID before acting.

## Prerequisites

- An AWS account with **GPU quota**: Service Quotas → EC2 → *Running On-Demand
  G and VT instances* ≥ 8 vCPUs in your region. New accounts often start at 0 and get a first request denied;
  an appeal with a small, specific ask and a description of cost controls
  worked here.
  - On a new account, choose the **Paid** plan at sign-up: the Free plan
    blocks GPU instance sizes. Free Tier credits still apply.
- AWS CLI v2 with a **named profile** for that account.
- Tailscale on the laptop, logged in to the tailnet the VM will join.
- The [Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/download.html)
  (2.0.0 works on Ubuntu 20.04).

## One-time setup

```bash
# 1. Settings. config.env is git-ignored; scripts refuse to run unless the
#    AWS credentials belong to EXPECTED_ACCOUNT_ID.
cp infra/config.env.example infra/config.env
$EDITOR infra/config.env

# 2. Create key pair, security group (SSH from your IP only) and the instance.
#    First boot installs the NVIDIA driver, Docker, the NVIDIA Container
#    Toolkit and Tailscale, then reboots once (~3 min).
infra/scripts/provision.sh

# 3. Join the VM to your tailnet with Tailscale SSH enabled.
ssh -t -i ~/.ssh/isaac-sim-ws.pem ubuntu@<public-ip> 'sudo tailscale up --ssh --hostname isaac-sim-ws'
#    In the Tailscale admin console: disable key expiry for isaac-sim-ws.

# 4. Close the temporary public SSH rule: the VM now has no inbound rules.
aws ec2 revoke-security-group-ingress --profile <profile> --region <region> \
  --group-id <sg-id> --protocol tcp --port 22 --cidr <your-ip>/32

# 5. Install the on-VM tools: isaac-run and the idle auto-shutdown timer.
infra/scripts/push-vm-tools.sh

# 6. Pull the Isaac Sim image once (32 GB; a few minutes inside AWS).
ssh ubuntu@isaac-sim-ws 'docker pull nvcr.io/nvidia/isaac-sim:6.1.0'
```

## Daily use

```bash
infra/scripts/start.sh      # start the VM and Isaac Sim, print when it's ready
                            # then connect the streaming client to the printed IP
infra/scripts/status.sh     # instance, tailnet, Isaac Sim, auto-shutdown state
infra/scripts/stop.sh       # stop Isaac Sim, then the instance
```

`start.sh --vm-only` starts the VM without Isaac Sim (for batch jobs).
`start.sh` never restarts an Isaac Sim that is already running, so it is safe
to run during a session. On the VM, `isaac-run` (re)starts Isaac Sim; extra
arguments go to Kit.

### Idle auto-shutdown

`isaac-idle.timer` runs a check every 5 minutes. The VM powers itself off
(and, because shutdown behaviour is *stop*, the instance stops) once **all**
of these have held for `IDLE_MINUTES` (default 30):

| Signal | Detected by |
|---|---|
| no streaming client | no established TCP on 49100 from a non-loopback peer |
| no SSH session | no `tailscaled be-child ssh` process, nothing on :22 |
| GPU not busy | 5-sample average below `GPU_BUSY_PCT` (30%) |
| no override | `/run/isaac-keepawake` absent (`sudo touch` it to keep a job alive) |

Settings: `/etc/default/isaac-idle` on the VM (`DRY_RUN=1` only logs).
Logs: `journalctl -t isaac-idle`.

## Measured results

| | |
|---|---|
| Cold start (empty caches) | **293 s** to "Full Streaming App is loaded" |
| Warm start, container restart | **44.5 s** |
| Warm start, after a full instance stop/start | **53 s** |
| Stream bandwidth while orbiting the viewport | **~2 Mbit/s** (≈ 0.9 GB per hour) |
| GPU use, Isaac Sim idle with a client connected | 6–7% |
| Bootstrap (driver, Docker, toolkit, Tailscale) | ~3 min including one reboot |

## Cost (us-east-2, on-demand, late 2026)

| Item | Price | Bills when |
|---|---|---|
| g6.2xlarge | ~$0.98/h | running |
| EBS gp3, 60 + 150 GB | ~$17/month | always, even when stopped |
| Public IPv4 | $0.005/h | running |
| Data out (stream) | first 100 GB/month free | ~110 streaming hours/month free |

At 20 GPU-hours a week that's roughly $100/month. Auto-shutdown is the main
protection: a forgotten weekend would otherwise cost ~$50.

## Lessons learned (gotchas)

1. **`--network=host` is required.** With `-p` port publishing, signalling
   connects but no video arrives: the media socket binds to `ISAACSIM_HOST`,
   which must be a real interface inside the container.
2. **Never open the streaming ports publicly.** No auth, no encryption, and a
   cloud public IP isn't on any interface anyway. Tailscale solves both.
3. **The container runs as UID 1234.** Mounted cache directories must be
   writable by that UID; `:rw` doesn't grant it.
4. **g6 instances have two kinds of NVMe disk.** EBS (persistent) and instance
   store (wiped on every stop) both appear as `/dev/nvme*n1`, in no fixed
   order. The bootstrap finds the data disk by its model string.
5. **`InsufficientInstanceCapacity` on start.** A stopped instance gives its
   GPU back; starting needs a free one in the same Availability Zone (the
   disks pin it there). It happened once here and cleared after ~40 min.
   `start.sh` keeps retrying the same type (`START_ROUNDS` attempts,
   `ROUND_WAIT` s apart; ~30 min by default) rather than falling back to
   another type.
6. **Docker's `data-root` doesn't move image layers.** Recent Docker stores
   them through containerd's image store under `/var/lib/containerd`. The
   32 GB image silently filled the root disk until containerd's own `root`
   was moved to `/data/containerd`, with `RequiresMountsFor=/data` on both
   services.
7. **Isaac Sim connects to its own signalling port** over loopback. An idle
   check counting "any connection on 49100" never fires; count only
   non-loopback peers. Found by testing the detector with the client closed.

Smaller ones: a stop with Isaac Sim running took ~7 minutes (`stop.sh` stops
the container first); `reboot` in a script returns immediately (add `exit 0`);
Tailscale's default SSH policy uses *check* mode, which asks for a browser
login every 12 h and blocks unattended scripts (set the rule to *accept* on a
personal tailnet).

## Troubleshooting

- **Client connects but the screen stays black.** Check the media socket and
  traffic on the VM: `ss -uln | grep 47998` and the `tailscale0` byte counters.
  Raise the streaming log level with
  `isaac-run --/log/channels/omni.kit.livestream.streamsdk=info`, then restart
  the client. A fresh server and client cleared it here.
- **`ssh isaac-sim-ws` hangs.** The VM is stopped (`status.sh`), or the laptop
  is on a different tailnet (`sudo tailscale switch <tailnet>`).
- **The instance won't stop.** `infra/scripts/stop.sh --force`.

## Teardown

The data volume is deliberately kept when the instance is terminated, so
delete it explicitly or it keeps billing:

```bash
aws ec2 terminate-instances --instance-ids <instance-id>
aws ec2 describe-volumes --filters Name=tag:Name,Values=isaac-sim-ws   # find the data volume
aws ec2 delete-volume --volume-id <data-volume-id>
aws ec2 delete-security-group --group-id <sg-id>
aws ec2 delete-key-pair --key-name isaac-sim-ws
```
