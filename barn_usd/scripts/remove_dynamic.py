"""
SURV-10: remove moving things (the person walking with the robot) from the map.

Ray casting on a voxel grid, using the raw bag:
  /bievr_lio/points/registered   each scan, already registered (odom frame)
  /bievr_lio/odom                sensor pose per scan
  /tf map->odom                  small global correction (the PCD is in the map frame)
Every ray marks the voxels it passes through as "free" and its end voxel as "hit".
A person occupies a voxel for a second or two and is then seen through by later
rays; walls, posts and equipment keep getting hit. Points in voxels that are
mostly seen through are dropped.

Pass 1 accumulates free/hit counts; pass 2 rebuilds the map from the scans in the
barn frame (same BarnFrame correction as flatten_map.py) without dynamic points.

Usage (venv active, from barn_usd/):
    python scripts/remove_dynamic.py barns/farm1_barn1 barn1full-1970-01-02-00-25-28 \
        barn1full_map_consistent_flatten_report.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

sys.path.insert(0, str(Path(__file__).parent))
from flatten_map import BarnFrame, new_las, write_points  # noqa: E402

CLOUD, ODOM, TF = "/bievr_lio/points/registered", "/bievr_lio/odom", "/tf"
LIDAR_IN_IMU = np.array([-0.011, -0.02329, 0.04412])  # tf_static mid360_imu -> mid360_lidar


def quat_rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    u, w = np.asarray(q[:3]), q[3]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def read_bag(bag_dir: Path):
    """Yield (scan xyz+intensity in map frame, lidar origin in map frame) per scan."""
    files = sorted(bag_dir.glob("*.mcap"))
    map_odom = np.zeros(3)  # latest map->odom translation (rotation is ~identity: < 0.04 deg)
    clouds = {}  # each cloud arrives just before the odom message with the same stamp
    stamp = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)  # noqa: E731
    for f in files:
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, ch, _, m in reader.iter_decoded_messages(topics=[CLOUD, ODOM, TF]):
                if ch.topic == TF:
                    for t in m.transforms:
                        if t.header.frame_id == "map" and t.child_frame_id == "odom":
                            a = t.transform.translation
                            map_odom = np.array([a.x, a.y, a.z])
                elif ch.topic == CLOUD:
                    clouds[stamp(m)] = np.frombuffer(bytes(m.data), dtype="<f4").reshape(
                        -1, m.point_step // 4)[:, :4]
                elif stamp(m) in clouds:
                    p, q = m.pose.pose.position, m.pose.pose.orientation
                    origin = np.array([p.x, p.y, p.z]) + quat_rotate((q.x, q.y, q.z, q.w), LIDAR_IN_IMU) + map_odom
                    xyzi = clouds.pop(stamp(m)).astype(np.float64)
                    xyzi[:, :3] += map_odom
                    yield xyzi, origin


class Grid:
    """Dense voxel grid over the barn, only the band where people walk."""

    def __init__(self, voxel=0.1, x=(-1.5, 155.5), y=(-8.5, 5.5), z=(0.15, 2.35)):
        self.v = voxel
        self.lo = np.array([x[0], y[0], z[0]])
        self.shape = np.ceil((np.array([x[1], y[1], z[1]]) - self.lo) / voxel).astype(int)
        self.size = int(np.prod(self.shape))
        self.free = np.zeros(self.size, np.int32)
        self.hit = np.zeros(self.size, np.int32)

    def index(self, p):
        """Flat voxel index per point, -1 outside the grid."""
        ijk = np.floor((p - self.lo) / self.v).astype(np.int64)
        ok = np.all((ijk >= 0) & (ijk < self.shape), axis=1)
        flat = np.full(len(p), -1, np.int64)
        s = self.shape
        flat[ok] = (ijk[ok, 0] * s[1] + ijk[ok, 1]) * s[2] + ijk[ok, 2]
        return flat

    def add(self, pts, origin, rng, step=0.05, stop_short=0.25, max_range=25.0, stride=2):
        """Accumulate one scan: free along each ray (up to stop_short before the end), hit at the end."""
        pts = pts[::stride]
        d = pts - origin
        L = np.linalg.norm(d, axis=1)
        keep = (L > 0.5) & (L < max_range)
        d, L, pts = d[keep] / L[keep, None], L[keep], pts[keep]
        h = self.index(pts)
        self.hit += np.bincount(h[h >= 0], minlength=self.size).astype(np.int32)
        k = np.floor((L - stop_short) / step).astype(np.int64)
        k = np.clip(k, 0, None)
        ray = np.repeat(np.arange(len(L)), k)
        start = np.repeat(np.cumsum(k) - k, k)
        t = (np.arange(k.sum()) - start) * step + rng.uniform(0, step, k.sum())
        samples = origin + d[ray] * t[:, None]
        f = self.index(samples)
        # a straight ray never re-enters a voxel, so repeats are consecutive: keep the first
        # of each run so each ray frees a voxel at most once
        first = np.r_[True, (f[1:] != f[:-1]) | (ray[1:] != ray[:-1])]
        f = f[first & (f >= 0)]
        self.free += np.bincount(f, minlength=self.size).astype(np.int32)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    ap.add_argument("bag", help="bag folder inside <barn_dir>/survey/map/")
    ap.add_argument("report", help="flatten report inside <barn_dir>/survey/ (yaw + floor curve)")
    ap.add_argument("--min-free", type=int, default=5, help="seen-through at least this many times")
    ap.add_argument("--ratio", type=float, default=1.0, help="... and more than ratio x hits")
    ap.add_argument("--split-height", type=float, default=2.5)
    args = ap.parse_args()

    survey = args.barn_dir / "survey"
    corr = json.loads((survey / args.report).read_text())["correction"]
    frame = BarnFrame(np.radians(corr["yaw_deg"]), corr["floor_poly_highest_power_first"], (-2.0, 157.0))
    bag = survey / "map" / args.bag

    grid, rng = Grid(), np.random.default_rng(0)
    print("pass 1: ray casting")
    for i, (xyzi, origin) in enumerate(read_bag(bag)):
        pts = frame.apply(xyzi[:, :3])
        o = frame.apply(origin[None, :])[0]
        grid.add(pts, o, rng)
        if i % 200 == 0:
            print(f"  scan {i}", end="\r")
    dynamic = (grid.free >= args.min_free) & (grid.free > args.ratio * grid.hit)
    print(f"\n{dynamic.sum():,} of {(grid.hit > 0).sum():,} occupied voxels are dynamic")
    np.savez_compressed(survey / "map" / "dynamic_grid.npz", free=grid.free, hit=grid.hit,
                        lo=grid.lo, shape=grid.shape, voxel=grid.v)

    print("pass 2: rebuilding the map without dynamic points")
    stem = args.bag
    outs = {k: survey / "map" / f"{stem}_barn_clean{k}.las" for k in ("", "_bottom", "_top", "_removed")}
    writers = {k: new_las(p) for k, p in outs.items()}
    kept = removed = 0
    try:
        for xyzi, _ in read_bag(bag):
            pts = frame.apply(xyzi[:, :3])
            idx = grid.index(pts)
            dyn = np.zeros(len(pts), bool)
            dyn[idx >= 0] = dynamic[idx[idx >= 0]]
            s, inten = pts[~dyn], xyzi[~dyn, 3]
            low = s[:, 2] <= args.split_height
            write_points(writers[""], s, inten)
            write_points(writers["_bottom"], s[low], inten[low])
            write_points(writers["_top"], s[~low], inten[~low])
            write_points(writers["_removed"], pts[dyn], xyzi[dyn, 3])
            kept += len(s)
            removed += dyn.sum()
    finally:
        for w in writers.values():
            w.close()
    print(f"kept {kept:,}, removed {removed:,} ({100 * removed / (kept + removed):.2f}%)")
    for p in outs.values():
        print("  ->", p.name)


if __name__ == "__main__":
    main()
