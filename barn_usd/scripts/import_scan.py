"""
SURV-09: bring the corrected lidar map into USD as a reference overlay.

  survey/map/<cloud>.las  (barn frame, from flatten_map.py)
    -> scan/tile_NNN.usdc   one UsdGeom.Points per x-slice, voxel-downsampled,
                            intensity as a primvar and as grey displayColor
    -> layers/scan.usda     /World/Scan/Tile_NNN, each tile a payload,
                            purpose = guide (never in final renders or training images)

Tiles are .usdc (binary): millions of points as text would be unusable. They are
git-ignored like the map; re-run this script to rebuild them.

Usage (venv active, from barn_usd/):
    python scripts/import_scan.py barns/farm1_barn1 barn1full_map_consistent_barn.las
    usdview barns/farm1_barn1/layers/scan.usda      # then View > Display Purposes > Guide
"""
import argparse
import sys
from pathlib import Path

import laspy
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, Vt

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402


def read_las(path: Path):
    """All points as float32 xyz (barn frame, metres) + float32 intensity."""
    with laspy.open(str(path)) as f:
        n = f.header.point_count
        xyz = np.empty((n, 3), np.float32)
        inten = np.empty(n, np.float32)
        i = 0
        for pts in f.chunk_iterator(5_000_000):
            k = len(pts)
            xyz[i:i + k, 0], xyz[i:i + k, 1], xyz[i:i + k, 2] = pts.x, pts.y, pts.z
            inten[i:i + k] = pts.intensity
            i += k
    return xyz, inten


def voxel_mean(xyz, inten, voxel):
    """One point per voxel cube: the mean position and intensity of the points in it."""
    keys = np.floor(xyz / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, inv, counts = np.unique(flat, return_inverse=True, return_counts=True)
    out = np.empty((len(counts), 3), np.float32)
    for d in range(3):
        out[:, d] = np.bincount(inv, weights=xyz[:, d], minlength=len(counts)) / counts
    oi = (np.bincount(inv, weights=inten, minlength=len(counts)) / counts).astype(np.float32)
    return out, oi


def write_tile(path: Path, name: str, xyz, inten, grey, width):
    """One tile file: defaultPrim /<name> (Xform) with a Points child."""
    if path.exists():
        path.unlink()  # CreateNew refuses to overwrite
    stage = usd_utils.new_layer(path, default_prim=name, prim_type="Xform")
    pts = UsdGeom.Points.Define(stage, f"/{name}/Points")
    pts.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(xyz))
    # one width for every point (constant interpolation): the voxel size, so dots just touch
    pts.CreateWidthsAttr(Vt.FloatArray([width]))
    pts.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    # extent lets viewers cull a tile without reading its points
    pts.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*xyz.min(0).tolist()), Gf.Vec3f(*xyz.max(0).tolist())]))
    # raw intensity (0-255) for analysis, and a grey displayColor so every viewer shows it
    pv = UsdGeom.PrimvarsAPI(pts)
    pv.CreatePrimvar("intensity", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.vertex).Set(
        Vt.FloatArray.FromNumpy(inten))
    pts.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
        Vt.Vec3fArray.FromNumpy(np.repeat(grey[:, None], 3, axis=1)))
    stage.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path, help="e.g. barns/farm1_barn1")
    ap.add_argument("las", help="corrected cloud inside <barn_dir>/survey/map/")
    ap.add_argument("--voxel", type=float, default=0.03, help="downsample cube size, m (default 0.03)")
    ap.add_argument("--tile", type=float, default=10.0, help="tile length along x, m (default 10)")
    args = ap.parse_args()

    xyz, inten = read_las(args.barn_dir / "survey" / "map" / args.las)
    print(f"{len(xyz):,} points read")

    # grey level: intensity scaled to its 1st-99th percentile, then gamma to lift dark surfaces
    lo, hi = np.percentile(inten[::100], [1, 99])

    # sort once by tile index so each tile is a contiguous slice
    x0 = np.floor(xyz[:, 0].min() / args.tile) * args.tile
    tile_idx = ((xyz[:, 0] - x0) // args.tile).astype(np.int32)
    order = np.argsort(tile_idx, kind="stable")
    xyz, inten, tile_idx = xyz[order], inten[order], tile_idx[order]
    bounds = np.searchsorted(tile_idx, np.arange(tile_idx.max() + 2))

    scan_dir = args.barn_dir / "scan"
    layer_path = args.barn_dir / "layers" / "scan.usda"
    if layer_path.exists():
        layer_path.unlink()
    stage = usd_utils.new_layer(layer_path)  # /World, Z-up, metres
    scan = UsdGeom.Xform.Define(stage, "/World/Scan")
    # guide purpose is inherited by every tile under /World/Scan
    scan.CreatePurposeAttr(UsdGeom.Tokens.guide)
    scan.GetPrim().SetCustomDataByKey("barn:source", args.las)
    scan.GetPrim().SetCustomDataByKey("barn:voxelM", args.voxel)

    total = 0
    for t in range(len(bounds) - 1):
        a, b = bounds[t], bounds[t + 1]
        if b - a == 0:
            continue
        txyz, tint = voxel_mean(xyz[a:b], inten[a:b], args.voxel)
        grey = (np.clip((tint - lo) / (hi - lo), 0, 1) ** 0.6).astype(np.float32)
        name = f"Tile_{t:03d}"
        tile_path = scan_dir / f"tile_{t:03d}.usdc"
        write_tile(tile_path, name, txyz, tint, grey, args.voxel)

        # the payload: geometry loads only when the tile is loaded
        prim = stage.DefinePrim(f"/World/Scan/{name}", "Xform")
        prim.GetPayloads().AddPayload(f"../scan/{tile_path.name}")  # relative to scan.usda
        prim.SetCustomDataByKey("barn:xRangeM", Gf.Vec2d(x0 + t * args.tile, x0 + (t + 1) * args.tile))
        total += len(txyz)
        print(f"  {name}: {b - a:>10,} -> {len(txyz):>9,} points")

    stage.GetRootLayer().Save()
    print(f"{total:,} points in {len(scan.GetPrim().GetChildren())} tiles -> {layer_path}")


if __name__ == "__main__":
    main()
