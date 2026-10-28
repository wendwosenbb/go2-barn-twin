# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A learning-oriented project for building a digital twin of a barn (`go2-barn-twin`) in OpenUSD, to be simulated in NVIDIA Isaac Sim 6.1 running on a cloud GPU. It is organized as course-style "units"/lessons; much of it is work in progress. There is no build system, test suite, or linter yet.

Two independent halves:

- **Local OpenUSD authoring** (laptop: Ubuntu 20.04, GCC 9, Python 3.8): `barn_usd/` (the barn scene; `scratch/` holds experiments) and `usd-tutorial/` (Pixar tutorial exercises). Python scripts author `.usda` stages via `pxr`.
- **Cloud Isaac Sim workstation** (`infra/`): bash + AWS CLI scripts that provision an EC2 `g6.2xlarge` running Isaac Sim headless, streamed over WebRTC through Tailscale. See `infra/README.md`.

`books_resources/` holds reference PDFs; `third_party/` is git-ignored.

## Python / OpenUSD environment

- Use the repo venv: `source .venv/bin/activate`. OpenUSD is **built from source** (not pip `usd-core`) into `~/openusd_built`, and the venv's `activate` script prepends its `bin` and `lib/python` to `PATH`/`PYTHONPATH`. If `pxr` imports fail or `usdview` is missing, the venv likely wasn't re-activated after a build.
- Rebuild OpenUSD with `bash infra/build_openusd.sh` (venv active; the script's usage comment says `tools/`, but it lives in `infra/`). It pins OpenUSD `v25.08` (newest that compiles with GCC 9), `cmake<4`, and `-j 3` (higher froze the laptop). It takes ~1–2 h, and leaves `~/openusd_built` read-only.
- Run an authoring script from the directory where its output should land, because scripts use relative paths in `Usd.Stage.CreateNew(...)`: e.g. `cd barn_usd/scratch && python hello_barn.py`. View the result with `usdview <file>.usda`.

## Scene conventions (barn_usd)

Follow these because the target is Isaac Sim/ROS:
- Stage `upAxis = Z` and `metersPerUnit = 1.0`. USD's fallback is centimeters, which causes 100× scale bugs.
- `/World` is an `Xform` and the stage's `defaultPrim`.
- Place objects so they rest on the Z=0 floor plane (account for centered prim origins).
- Custom attributes are namespaced `barn:` (e.g. `barn:lineIndex`).
- Existing scripts comment nearly every line to explain the USD concept involved; match that style in tutorial/scratch code.

## infra/ (AWS)

- Copy `infra/config.env.example` to `infra/config.env` (git-ignored) and fill it in. Every script sources it through `infra/scripts/lib.sh`.
- `lib.sh` provides `die`, `info`, `guard_account`, and `instance_id`. **Every script must call `guard_account`** before touching AWS, so a GPU instance can never be created in the wrong account (the guard checks `EXPECTED_ACCOUNT_ID` via STS). New scripts in `infra/scripts/` should `source "$(dirname "$0")/lib.sh"` and then call `guard_account`.
- `infra/scripts/provision.sh` runs once to create the key pair, an SSH-only-from-your-IP security group, and the instance. The instance has two EBS volumes: a disposable root, and a data volume at `/data` with `DeleteOnTermination=false` that holds Docker and the Isaac Sim caches.
- `infra/bootstrap/user-data.sh` is the cloud-init first-boot script (NVIDIA driver 595, Docker with its data-root on `/data/docker`, the NVIDIA Container Toolkit, and Tailscale). It must stay **idempotent**. It finds the data disk by EBS model name, never by device number. Isaac Sim cache directories under `/data/isaac-sim` are owned by container UID 1234.
- Never open Isaac Sim/WebRTC ports publicly. Streaming is unauthenticated, so it goes only over Tailscale.
- Scripts use `set -euo pipefail` and include shellcheck directives. Check them with `shellcheck infra/scripts/*.sh infra/bootstrap/*.sh` if it is installed.
