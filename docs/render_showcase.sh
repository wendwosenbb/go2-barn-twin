#!/usr/bin/env bash
# Render the README images. Run from the repo root with the venv active;
# the scan image needs the scan tiles (scripts/import_scan.py) on disk.
set -euo pipefail
cd "$(dirname "$0")"
usdrecord --camera /World/PanCam --imageWidth 1000 showcase.usda images/feeder_pan.png
usdrecord --camera /World/DrinkerCam --imageWidth 1000 showcase.usda images/drinker_nipple.png
usdrecord --camera /World/ScanBird --purposes guide --imageWidth 1600 showcase.usda images/scan_birdseye.png
usdrecord --camera /World/ScanCutaway --purposes guide --imageWidth 1600 showcase.usda images/scan_cutaway.png
