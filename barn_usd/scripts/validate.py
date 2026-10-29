"""
PIPE-03: validate.py v1 — the checks of design section 11 on a composed stage.

  metadata    every layer in the stack: upAxis Z, metersPerUnit 1, defaultPrim set
  paths       no unresolved asset paths (UsdUtils.ComputeAllDependencies)
  kinds       every component has assetInfo, sits in a valid model hierarchy
              (kinds set on a prim that is not a model), and has no component inside it
  floor       every component / loose gprim (non-guide) has world bbox min.z >= -0.01,
              except prims marked barn:allowBelowFloor (the floor slab)
  instancing  every instance has a prototype; prototype count stays small

Exit code 1 if any check fails. The sim checks (one PhysicsScene, colliders on
floor/walls) come with SIM-01.

Usage (venv active, from barn_usd/):
    python scripts/validate.py barns/farm1_barn1/barn.usda
"""
import argparse
import sys

from pxr import Sdf, Usd, UsdGeom, UsdUtils

FLOOR_TOL = -0.01
MAX_PROTOTYPES = 50


def check_metadata(stage):
    errs = []
    for layer in stage.GetUsedLayers():
        if layer.anonymous:
            continue
        root = layer.pseudoRoot
        name = layer.identifier
        if root.GetInfo("upAxis") != "Z":
            errs.append(f"{name}: upAxis is {root.GetInfo('upAxis')!r}, not 'Z'")
        if not root.HasInfo("metersPerUnit") or root.GetInfo("metersPerUnit") != 1.0:
            errs.append(f"{name}: metersPerUnit is {root.GetInfo('metersPerUnit') if root.HasInfo('metersPerUnit') else 'unset (USD falls back to 0.01)'}")
        if not layer.defaultPrim:
            errs.append(f"{name}: no defaultPrim")
    return errs


def check_paths(stage):
    _, _, unresolved = UsdUtils.ComputeAllDependencies(stage.GetRootLayer().identifier)
    return [f"unresolved asset path: {p}" for p in unresolved]


def _walk(stage):
    """Every prim, including the inside of instances (as instance proxies)."""
    return Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())


def check_kinds(stage):
    errs = []
    for prim in _walk(stage):
        kind = Usd.ModelAPI(prim).GetKind()
        if not kind:
            continue
        if kind != "subcomponent" and not prim.IsModel():
            errs.append(f"{prim.GetPath()}: kind '{kind}' but not in a valid model hierarchy "
                        "(every ancestor needs group/assembly)")
        if kind == "component":
            if not prim.GetAssetInfo():
                errs.append(f"{prim.GetPath()}: component without assetInfo")
            parent = prim.GetParent()
            while parent and not parent.IsPseudoRoot():
                if Usd.ModelAPI(parent).GetKind() == "component":
                    errs.append(f"{prim.GetPath()}: component nested in component {parent.GetPath()}")
                    break
                parent = parent.GetParent()
    return errs


def _allowed_below(prim):
    while prim and not prim.IsPseudoRoot():
        a = prim.GetAttribute("barn:allowBelowFloor")
        if a and a.Get():
            return True
        prim = prim.GetParent()
    return False


def check_floor(stage):
    errs = []
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    it = iter(_walk(stage))
    for prim in it:
        is_component = Usd.ModelAPI(prim).GetKind() == "component"
        if not (is_component or prim.IsA(UsdGeom.Gprim)):
            continue
        it.PruneChildren()  # judge a component as a whole, not part by part
        if _allowed_below(prim):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not box.IsEmpty() and box.GetMin()[2] < FLOOR_TOL:
            errs.append(f"{prim.GetPath()}: bottom at z = {box.GetMin()[2]:.3f} m (below the floor)")
    return errs


def check_instancing(stage):
    errs = []
    protos = stage.GetPrototypes()
    if len(protos) > MAX_PROTOTYPES:
        errs.append(f"{len(protos)} prototypes (> {MAX_PROTOTYPES}): instances are not sharing")
    for prim in stage.Traverse():
        if prim.IsInstance() and not prim.GetPrototype():
            errs.append(f"{prim.GetPath()}: instance without a prototype")
    return errs


CHECKS = {"metadata": check_metadata, "paths": check_paths, "kinds": check_kinds,
          "floor": check_floor, "instancing": check_instancing}


def validate(path, verbose=True):
    """Run every check; return {check: [errors]}."""
    stage = Usd.Stage.Open(str(path), Usd.Stage.LoadAll)
    results = {name: fn(stage) for name, fn in CHECKS.items()}
    if verbose:
        for name, errs in results.items():
            print(f"{'FAIL' if errs else 'ok  '} {name}" + (f"  ({len(errs)})" if errs else ""))
            for e in errs[:10]:
                print("       ", e)
            if len(errs) > 10:
                print(f"        ... and {len(errs) - 10} more")
        print(f"{len(stage.GetPrototypes())} prototypes, "
              f"{sum(1 for p in stage.Traverse() if p.IsInstance())} instances")
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", help="entry .usda (e.g. barns/<id>/barn.usda) or any layer")
    args = ap.parse_args()
    results = validate(args.stage)
    sys.exit(1 if any(results.values()) else 0)


if __name__ == "__main__":
    main()
