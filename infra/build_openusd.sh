#!/usr/bin/env bash
# build_openusd.sh — rebuild OpenUSD with Hydra/usdview on Ubuntu 20.04 (GCC 9, Python 3.8)
# Recipe verified Oct 7 2026 on the Crosshair 15 laptop (RTX 3060, 15 GB RAM).
# Usage (from anywhere, with the course venv active): bash tools/build_openusd.sh

# Stop at the first failing command, on unset variables, and on failures inside pipes
set -euo pipefail

# OpenUSD release to build: v25.08 is the newest that compiles with GCC 9 (v25.11+ adds -Wmismatched-tags)
USD_TAG="v25.08"
# Where Pixar's source code is checked out
SRC_DIR="$HOME/src/OpenUSD"
# Where the finished install goes (the venv's activate script points here)
INSTALL_DIR="$HOME/openusd_built"
# Parallel compile jobs: 3 keeps peak RAM around 5–6 GB; 8+ froze the laptop
JOBS=3

# Refuse to run outside a venv, so the build links against the venv's Python
if [ -z "${VIRTUAL_ENV:-}" ]; then echo "Activate the course venv first: source ~/go2-barn-twin/.venv/bin/activate"; exit 1; fi

# System headers needed by Hydra Storm / usdview (adds files only; replaces no system packages)
sudo apt-get install -y git build-essential libx11-dev libxt-dev libgl1-mesa-dev libglu1-mesa-dev

# Remove pip's usd-core so it can't shadow the built pxr module
pip uninstall -y usd-core || true
# Build tools + usdview dependencies; CMake pinned below 4 (4.x rejects old dependency CMake files)
pip install "cmake<4" PySide2 PyOpenGL numpy jinja2

# Clone the source if it isn't there yet
if [ ! -d "$SRC_DIR" ]; then git clone https://github.com/PixarAnimationStudios/OpenUSD "$SRC_DIR"; fi
# Fetch the pinned release tag (works on shallow clones too)
git -C "$SRC_DIR" fetch --depth 1 origin tag "$USD_TAG"
# Switch the source to that release
git -C "$SRC_DIR" checkout "$USD_TAG"

# If a previous install was made read-only, unlock it so the build can write into it
if [ -d "$INSTALL_DIR" ]; then chmod -R u+w "$INSTALL_DIR"; fi

# Build and install: dependencies (TBB, MaterialX, OpenSubdiv), then USD with imaging + usdview (~1–2 h at -j 3)
python "$SRC_DIR/build_scripts/build_usd.py" -j "$JOBS" "$INSTALL_DIR"

# Free several GB: intermediate objects and downloaded dependency sources aren't needed after install
rm -rf "$INSTALL_DIR/build" "$INSTALL_DIR/src"
# Make the install read-only, so an accidental rm -rf fails
chmod -R a-w "$INSTALL_DIR"

# Add the install to the venv's activate script, only if it isn't there already
grep -q openusd_built "$VIRTUAL_ENV/bin/activate" || echo 'export PATH="$HOME/openusd_built/bin:$PATH" PYTHONPATH="$HOME/openusd_built/lib/python:$PYTHONPATH"' >> "$VIRTUAL_ENV/bin/activate"

# Final reminder: the current shell still has the old paths until the venv is re-activated
echo "Done. Run: deactivate; source $VIRTUAL_ENV/bin/activate && usdview <file.usda>"
