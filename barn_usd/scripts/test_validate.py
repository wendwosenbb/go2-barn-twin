"""
PIPE-03 acceptance: validate.py passes a good layer and fails deliberately broken ones,
each on the check it breaks and only that one.

Usage (venv active, from barn_usd/):
    python scripts/test_validate.py
"""
import sys
import tempfile
from pathlib import Path

from pxr import Gf, Kind, Sdf, Usd, UsdGeom

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402
from validate import validate  # noqa: E402


def make(path, *, meters=True, missing_ref=False, bad_kind=False, below=False):
    stage = usd_utils.new_layer(path)
    if not meters:
        stage.GetRootLayer().pseudoRoot.ClearInfo("metersPerUnit")
    Usd.ModelAPI(stage.GetPrimAtPath("/World")).SetKind(Kind.Tokens.assembly)
    cube = UsdGeom.Cube.Define(stage, "/World/Box")
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0, 0, -0.5 if below else 0.5))
    if missing_ref:
        stage.DefinePrim("/World/Ghost").GetReferences().AddReference("./no_such_asset.usda")
    if bad_kind:
        # a component under an untyped, kind-less parent breaks the model hierarchy,
        # and it has no assetInfo either
        stage.DefinePrim("/World/Loose")
        Usd.ModelAPI(stage.DefinePrim("/World/Loose/Thing", "Xform")).SetKind(Kind.Tokens.component)
    stage.GetRootLayer().Save()


CASES = {  # name: (make() options, the checks that must fail)
    "good": ({}, set()),
    "centimetres": ({"meters": False}, {"metadata"}),
    "missing_ref": ({"missing_ref": True}, {"paths"}),
    "bad_kind": ({"bad_kind": True}, {"kinds"}),
    "below_floor": ({"below": True}, {"floor"}),
}


def main():
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for name, (opts, expect) in CASES.items():
            path = Path(tmp) / f"{name}.usda"
            make(path, **opts)
            failed = {check for check, errs in validate(path, verbose=False).items() if errs}
            good = failed == expect
            ok &= good
            print(f"{'PASS' if good else 'FAIL'} {name:12s} failed checks: {sorted(failed) or '-'}"
                  f"{'' if good else f'  (expected {sorted(expect) or None})'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
