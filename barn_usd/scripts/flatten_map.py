"""
SURV-06: straighten a drifted lidar map into the barn frame.

The LIO map of a flat barn bends: the floor drops along x (pitch/z drift) and the
robot started a little off the wall direction (yaw). This script
  1. finds the yaw that makes the long walls parallel to x,
  2. fits the floor height along x, f(x), on floor-only points,
  3. unbends every point: height above the fitted floor becomes z, arc length
     along the floor becomes x, so floor -> Z=0 and walls/posts stand vertical,
  4. writes the corrected cloud (full + bottom/top split at a true height) as LAS
     and a JSON report with the numbers for barn.yaml.

Usage (venv active, from barn_usd/):
    python scripts/flatten_map.py barns/farm1_barn1 barn1full_map_consistent.pcd
"""
import argparse
import json
from pathlib import Path

import laspy
import numpy as np

CHUNK = 5_000_000  # points per processing chunk; keeps RAM use to a few hundred MB


# ---------------------------------------------------------------- input
def read_pcd(path: Path) -> np.ndarray:
    """Memory-map a binary PCD with float32 fields x y z intensity -> (N, 4) view."""
    with open(path, "rb") as fh:
        offset = 0
        fields = None
        while True:
            line = fh.readline()
            offset += len(line)
            if line.startswith(b"FIELDS"):
                fields = line.split()[1:]
            if line.startswith(b"DATA"):
                if b"binary" not in line or b"compressed" in line:
                    raise ValueError("only uncompressed binary PCD is supported")
                break
    if fields != [b"x", b"y", b"z", b"intensity"]:
        raise ValueError(f"expected fields x y z intensity, got {fields}")
    return np.memmap(path, dtype="<f4", mode="r", offset=offset).reshape(-1, 4)


# ---------------------------------------------------------------- yaw
def rotate_z(xy: np.ndarray, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.c_[c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]]


def estimate_yaw(xy: np.ndarray) -> float:
    """Yaw (rad) that lines the long walls up with x.

    When the walls are parallel to x, all wall points fall into a few y bins, so the
    y histogram is sharpest. Score = sum of squared bin counts; coarse then fine search.
    """
    def sharpness(yaw):
        y = rotate_z(xy, yaw)[:, 1]
        h, _ = np.histogram(y, bins=np.arange(y.min(), y.max(), 0.05))
        return float(np.sum(h.astype(np.float64) ** 2))

    best = 0.0
    for span, step in ((np.radians(10), np.radians(0.1)), (np.radians(0.2), np.radians(0.005))):
        cands = np.arange(best - span, best + span + step / 2, step)
        best = cands[np.argmax([sharpness(a) for a in cands])]
    return float(best)


def find_long_walls(y: np.ndarray, min_width=5.0):
    """The two densest y bins at least min_width apart -> (y_min_wall, y_max_wall)."""
    h, e = np.histogram(y, bins=np.arange(y.min(), y.max(), 0.05))
    order = np.argsort(h)[::-1]
    first = e[order[0]] + 0.025
    for i in order[1:]:
        if abs(e[i] + 0.025 - first) >= min_width:
            return tuple(sorted((first, e[i] + 0.025)))
    raise RuntimeError("could not find two long walls")


# ---------------------------------------------------------------- floor
def floor_profile(x, y, z, y_lo, y_hi, bin_m=1.0):
    """Floor height per x bin: the most common z among the lowest 30% of points
    between the walls (the floor is the densest low surface; equipment sits higher)."""
    inside = (y > y_lo) & (y < y_hi)
    x, z = x[inside], z[inside]
    edges = np.arange(np.floor(x.min()), np.ceil(x.max()) + bin_m, bin_m)
    idx = np.digitize(x, edges) - 1
    xc, fz = [], []
    for i in range(len(edges) - 1):
        zz = z[idx == i]
        if len(zz) < 200:
            continue
        low = zz[zz <= np.percentile(zz, 30)]
        h, e = np.histogram(low, bins=np.arange(low.min(), low.max() + 0.02, 0.02))
        xc.append(edges[i] + bin_m / 2)
        fz.append(e[np.argmax(h)] + 0.01)
    return np.array(xc), np.array(fz)


def fit_floor(xc, fz, deg=4, clip=3.0, iters=5):
    """Polynomial f(x) with sigma clipping, so bins that caught equipment are dropped."""
    keep = np.ones(len(xc), bool)
    for _ in range(iters):
        p = np.polyfit(xc[keep], fz[keep], deg)
        r = fz - np.polyval(p, xc)
        mad = 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep])))
        new = np.abs(r) < clip * max(mad, 0.01)
        if (new == keep).all():
            break
        keep = new
    return p, keep


# ---------------------------------------------------------------- correction
class BarnFrame:
    """Map frame -> barn frame: yaw about z through the origin, then unbend in x-z.

    Reusable for anything recorded in the map frame (e.g. real robot poses).
    """

    def __init__(self, yaw, floor_poly, x_range):
        self.yaw = yaw
        self.f = np.poly1d(floor_poly)
        self.df = self.f.deriv()
        self.f0 = float(self.f(0.0))
        # arc length table s(x) along the floor curve, s(0) = 0
        self._xs = np.linspace(x_range[0] - 5, x_range[1] + 5, 20001)
        ds = np.sqrt(1 + self.df(self._xs) ** 2)
        s = np.concatenate([[0], np.cumsum((ds[1:] + ds[:-1]) / 2 * np.diff(self._xs))])
        self._s = s - np.interp(0.0, self._xs, s)

    def apply(self, xyz: np.ndarray) -> np.ndarray:
        xy = rotate_z(xyz[:, :2].astype(np.float64), self.yaw)
        x, y, z = xy[:, 0], xy[:, 1], xyz[:, 2].astype(np.float64)
        # A point = floor point C(x0) + h * floor normal. Find foot x0 (one fixed-point step
        # is enough for slopes of a few degrees), then height h along the normal.
        th = np.arctan(self.df(x))
        x0 = x + (z - self.f(x)) * np.sin(th)
        th0 = np.arctan(self.df(x0))
        h = (z - self.f(x0)) * np.cos(th0)
        return np.c_[np.interp(x0, self._xs, self._s), y, h]

    def to_dict(self):
        return {
            "yaw_deg": float(np.degrees(self.yaw)),
            "floor_poly_highest_power_first": [float(c) for c in self.f.coeffs],
            "floor_z_at_origin_m": self.f0,
            "max_floor_slope_deg": float(np.degrees(np.arctan(np.abs(self.df(self._xs)).max()))),
        }


# ---------------------------------------------------------------- checks
def end_walls(x, z, lo, hi):
    """x of the densest bins near each end, measured in a z band -> (x_min_wall, x_max_wall)."""
    m = (z > lo) & (z < hi)
    xs = x[m]
    h, e = np.histogram(xs, bins=np.arange(xs.min(), xs.max(), 0.05))
    n = len(h) // 10  # look in the first and last tenth of the barn
    return e[np.argmax(h[:n])] + 0.025, e[len(h) - n + np.argmax(h[-n:])] + 0.025


def ceiling_profile(x, y, z, y_lo, y_hi):
    """99.5th percentile height across y (ridge in the middle, eaves at the walls)."""
    out = []
    for y0 in np.linspace(y_lo, y_hi, 9)[:-1]:
        m = (y > y0) & (y < y0 + (y_hi - y_lo) / 8)
        out.append([round(float(y0 + (y_hi - y_lo) / 16), 2), round(float(np.percentile(z[m], 99.5)), 2)])
    return out


# ---------------------------------------------------------------- output
def new_las(path: Path, scale=0.001):
    header = laspy.LasHeader(point_format=0, version="1.2")
    header.scales = np.array([scale] * 3)
    header.offsets = np.zeros(3)
    return laspy.open(str(path), mode="w", header=header)


def write_points(writer, xyz, inten):
    pts = laspy.ScaleAwarePointRecord.zeros(len(xyz), header=writer.header)
    pts.x, pts.y, pts.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    pts.intensity = np.clip(inten, 0, 65535).astype(np.uint16)
    writer.write_points(pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path, help="e.g. barns/farm1_barn1")
    ap.add_argument("pcd", help="file name inside <barn_dir>/survey/map/")
    ap.add_argument("--split-height", type=float, default=2.5,
                    help="bottom/top split, metres above the floor (default 2.5)")
    ap.add_argument("--sample", type=int, default=10, help="use every n-th point for fitting")
    args = ap.parse_args()

    map_dir = args.barn_dir / "survey" / "map"
    cloud = read_pcd(map_dir / args.pcd)
    print(f"{len(cloud):,} points")

    # 1-3: fit on a subsample
    sub = np.asarray(cloud[:: args.sample, :3], dtype=np.float64)
    yaw = estimate_yaw(sub[:: 10, :2])
    xy = rotate_z(sub[:, :2], yaw)
    y_lo, y_hi = find_long_walls(xy[:, 1])
    print(f"yaw {np.degrees(yaw):+.3f} deg, long walls at y = {y_lo:.2f}, {y_hi:.2f}")

    xc, fz = floor_profile(xy[:, 0], xy[:, 1], sub[:, 2], y_lo + 0.5, y_hi - 0.5)
    poly, keep = fit_floor(xc, fz)
    resid = fz - np.polyval(poly, xc)
    print(f"floor fit: {keep.sum()}/{len(keep)} bins kept, rms {resid[keep].std():.3f} m")

    frame = BarnFrame(yaw, poly, (xc.min(), xc.max()))

    # 4: check on the corrected subsample
    c = frame.apply(sub)
    cx, cy, cz = c[:, 0], c[:, 1], c[:, 2]
    xc2, fz2 = floor_profile(cx, cy, cz, y_lo + 0.5, y_hi - 0.5)
    wall_lo = end_walls(cx, cz, 0.3, 1.0)
    wall_hi = end_walls(cx, cz, 2.0, 2.7)
    report = {
        "input": args.pcd,
        "points": int(len(cloud)),
        "correction": frame.to_dict(),
        "long_walls_y_m": [round(y_lo, 3), round(y_hi, 3)],
        "width_m": round(y_hi - y_lo, 3),
        "end_walls_x_m_at_0.3-1.0m": [round(float(v), 3) for v in wall_lo],
        "end_walls_x_m_at_2.0-2.7m": [round(float(v), 3) for v in wall_hi],
        "length_m": round(float(wall_lo[1] - wall_lo[0]), 3),
        "floor_after_m": {
            "median": round(float(np.median(fz2)), 3),
            "p05": round(float(np.percentile(fz2, 5)), 3),
            "p95": round(float(np.percentile(fz2, 95)), 3),
            "by_10m": [[round(float(a), 1), round(float(b), 3)] for a, b in zip(xc2[::10], fz2[::10])],
        },
        "ceiling_across_y_m": ceiling_profile(cx, cy, cz, y_lo, y_hi),
        "split_height_m": args.split_height,
    }

    # write the full corrected cloud plus the bottom/top split, chunk by chunk
    stem = Path(args.pcd).stem
    outs = {k: map_dir / f"{stem}_barn{k}.las" for k in ("", "_bottom", "_top")}
    writers = {k: new_las(p) for k, p in outs.items()}
    try:
        for i in range(0, len(cloud), CHUNK):
            chunk = np.asarray(cloud[i:i + CHUNK])
            xyz, inten = frame.apply(chunk[:, :3]), chunk[:, 3]
            low = xyz[:, 2] <= args.split_height
            write_points(writers[""], xyz, inten)
            write_points(writers["_bottom"], xyz[low], inten[low])
            write_points(writers["_top"], xyz[~low], inten[~low])
            print(f"  {min(i + CHUNK, len(cloud)):,} / {len(cloud):,}", end="\r")
    finally:
        for w in writers.values():
            w.close()
    print()
    report["outputs"] = [p.name for p in outs.values()]

    rpath = args.barn_dir / "survey" / f"{stem}_flatten_report.json"  # small: kept in git, unlike map/
    rpath.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "floor_after_m"}, indent=2))
    print("floor after:", {k: v for k, v in report["floor_after_m"].items() if k != "by_10m"})
    print(f"report -> {rpath}")


if __name__ == "__main__":
    main()
