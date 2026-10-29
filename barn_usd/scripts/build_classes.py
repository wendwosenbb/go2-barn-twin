"""
LIB-02: class prims for inherits -> library/classes.usda

One empty `class "_class_<AssetType>"` at the layer root per asset type in
library/equipment_types.csv, plus the items that repeat along lines. Barn layers
make prims inherit them, so one `over "_class_FeederPan"` (say, in a scenario or
a robot scene) reaches every pan. Class prims are abstract: never rendered.

Usage (venv active, from barn_usd/):
    python scripts/build_classes.py
"""
import csv
import sys
from pathlib import Path

from pxr import Sdf

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402

LIBRARY = Path(__file__).resolve().parents[1] / "library"
LINE_ITEMS = ["feeder_pan", "drinker_nipple"]


def class_name(asset_type: str) -> str:
    """feeder_pan -> _class_FeederPan"""
    return "_class_" + "".join(w.capitalize() for w in asset_type.split("_"))


def main():
    types = [r["asset_type"] for r in csv.DictReader(open(LIBRARY / "equipment_types.csv"))] + LINE_ITEMS
    path = LIBRARY / "classes.usda"
    if path.exists():
        path.unlink()
    # defaultPrim must exist; the first class serves (nothing references this file by default prim)
    stage = usd_utils.new_layer(path, default_prim=class_name(types[0]), prim_type="")
    stage.GetDefaultPrim().SetSpecifier(Sdf.SpecifierClass)  # new_layer made it a def
    for t in types[1:]:
        stage.CreateClassPrim(Sdf.Path("/" + class_name(t)))
    stage.GetRootLayer().Save()
    print(f"-> {path}  ({len(types)} classes)")


if __name__ == "__main__":
    main()
