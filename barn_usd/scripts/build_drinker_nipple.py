"""
LIB-04: drinker_nipple asset (nipple + drip cup) from the CAD model's STL.

Input: library/assets/drinker_nipple_pan/source/Nipple_Drinker_clean.stl (mm, Z-up).
The single mesh is split by height: drip cup (bottom 22 mm) and nipple + pipe clamp.
Pivot = bottom centre of the drip cup, matching how equipment.csv measures drinker
line height (lowest dense level of the cups).

Usage (venv active, from barn_usd/):
    python scripts/build_drinker_nipple.py
"""
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from make_asset import LIBRARY, make_asset  # noqa: E402

CUP_TOP_MM = 22.0


def read_binary_stl(path: Path):
    """Triangle soup -> (unique points (N,3), triangles (M,3))."""
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
    soup = np.frombuffer(data[84:84 + 50 * n], dtype=rec)["v"].reshape(-1, 3)
    pts, inv = np.unique(soup.round(4), axis=0, return_inverse=True)  # weld shared corners
    return pts.astype(np.float64), inv.reshape(-1, 3).astype(np.int32)


def split(pts, tris, keep):
    """Sub-mesh of the triangles where keep is True, with its own compact point list."""
    t = tris[keep]
    used, remap = np.unique(t, return_inverse=True)
    return pts[used], remap.reshape(-1, 3).astype(np.int32)


def main():
    src = LIBRARY / "assets" / "drinker_nipple_pan" / "source" / "Nipple_Drinker_clean.stl"
    pts, tris = read_binary_stl(src)
    centroid_z = pts[tris][:, :, 2].mean(axis=1)
    cup_pts = pts[pts[:, 2] < CUP_TOP_MM]
    # pivot: centre of the cup's footprint, on its bottom
    pivot = np.array([(cup_pts[:, 0].min() + cup_pts[:, 0].max()) / 2,
                      (cup_pts[:, 1].min() + cup_pts[:, 1].max()) / 2, pts[:, 2].min()])
    cup = split(pts, tris, centroid_z < CUP_TOP_MM)
    body = split(pts, tris, centroid_z >= CUP_TOP_MM)
    full = [{"name": "Cup", "mesh": ((cup[0] - pivot) * 0.001, cup[1]), "look": "Yellow"},
            {"name": "Nipple", "mesh": ((body[0] - pivot) * 0.001, body[1]), "look": "Grey"}]
    top = (pts[:, 2].max() - pivot[2]) * 0.001
    # nipple centre relative to the cup centre (the cup hangs on an arm, offset from the pipe)
    upper = pts[pts[:, 2] > 110] - pivot
    nx, ny = (upper[:, 0].min() + upper[:, 0].max()) / 2000, (upper[:, 1].min() + upper[:, 1].max()) / 2000

    proxy = [{"name": "Cup", "cylinder": (0.034, 0.022), "at": (0, 0, 0), "look": "Yellow"},
             {"name": "Nipple", "cylinder": (0.01, top - 0.07), "at": (nx, ny, 0.07), "look": "Grey"}]
    looks = {"Yellow": ("Plastic", {"diffuseColor": (0.95, 0.72, 0.05), "roughness": 0.45}),
             "Grey": ("Plastic", {"diffuseColor": (0.6, 0.61, 0.62)})}
    out = make_asset("drinker_nipple", "DrinkerNipple", "drinker_nipple", full, proxy, looks,
                     info={"barn:source": src.name, "barn:pivot": "bottom centre of the drip cup",
                           "barn:pipeAxis": "x"})
    print(f"-> {out}  ({len(tris):,} triangles, {top:.3f} m tall, nipple at ({nx:.3f}, {ny:.3f}))")


if __name__ == "__main__":
    main()
