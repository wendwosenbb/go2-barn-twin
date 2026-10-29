"""
BARN-01: build one barn -> barns/<barn_id>/barn.usda + barn_sim.usda

1. generated layers from the survey: layers/shell.usda (build_shell.py),
   layers/layout.usda (build_lines.py; BARN-03 adds the rest of equipment.csv)
2. every other layer of the stack created empty if missing; existing files are kept
   (sim_overrides.usda is hand-edited, the others get their builders later)
3. entry files with sublayer lists only, in design section 5's order (strongest first)
4. validate.py on barn.usda; a failed check fails the build

The scan overlay (layers/scan.usda, guide purpose) is added to barn.usda when present.

Usage (venv active, from barn_usd/):
    python scripts/build_barn.py barns/farm1_barn1
"""
import argparse
import subprocess
import sys
from pathlib import Path

from pxr import Sdf

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402
from validate import validate  # noqa: E402

SCRIPTS = Path(__file__).parent
RENDER_STACK = ["lighting", "looks", "dressing", "layout", "shell"]  # strongest first
SIM_STACK = ["sim_overrides", "physics", "semantics"]
CLASSES = "../../library/classes.usda"  # relative to barns/<id>/


def entry_layer(path: Path, sublayers):
    """A layer with stage metadata and sublayers, no prims."""
    if path.exists():
        path.unlink()
    layer = Sdf.Layer.CreateNew(str(path))
    layer.defaultPrim = "World"
    layer.pseudoRoot.SetInfo("upAxis", "Z")
    layer.pseudoRoot.SetInfo("metersPerUnit", 1.0)
    for sub in sublayers:
        layer.subLayerPaths.append(sub)
    layer.Save()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    args = ap.parse_args()
    barn, layers = args.barn_dir, args.barn_dir / "layers"

    for script in ("build_shell.py", "build_lines.py"):
        print(f"== {script}")
        subprocess.run([sys.executable, str(SCRIPTS / script), str(barn)], check=True)

    for name in RENDER_STACK + SIM_STACK:
        path = layers / f"{name}.usda"
        if not path.exists():
            stage = usd_utils.new_layer(path)  # keep the stage alive until saved
            stage.GetRootLayer().Save()
            print(f"   created empty {path}")

    render = [f"./layers/{n}.usda" for n in RENDER_STACK]
    if (layers / "scan.usda").exists():
        render.append("./layers/scan.usda")
    entry_layer(barn / "barn.usda", render + [CLASSES])
    entry_layer(barn / "barn_sim.usda", [f"./layers/{n}.usda" for n in SIM_STACK] + ["./barn.usda"])
    print(f"-> {barn / 'barn.usda'}\n-> {barn / 'barn_sim.usda'}")

    print("== validate.py barn.usda")
    results = validate(barn / "barn.usda")
    sys.exit(1 if any(results.values()) else 0)


if __name__ == "__main__":
    main()
