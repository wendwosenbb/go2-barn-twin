# Unit 3, Lesson 1 (theory, no GPU): repetitive vs non-repetitive lidar scan patterns.
#
# A lidar is, first of all, a list of BEAM DIRECTIONS fired at known TIMES. Before any physics,
# that list decides what the sensor can see. This script builds two patterns and measures how
# much of the field of view (FOV) each one covers as you integrate (accumulate) longer:
#
#   repetitive     "VLP-16 class" spinning lidar: 16 lasers at fixed elevations, spinning at
#                  10 Hz. Every revolution fires the SAME directions again.
#   non-repetitive "Mid-360 class": 200,000 points/s over 360 x (-7..52) deg. Directions keep
#                  changing, so coverage grows with integration time.
#
# IMPORTANT: Livox doesn't publish the Mid-360's pattern in its docs. The built-in one here is a
# PROXY with the right budget (point rate, FOV) and the right PROPERTY (it never repeats), but
# the WRONG SHAPE. The measured pattern used by the community Livox Gazebo plugins
# (scan_mode/mid360.csv, see load_csv) shows the real shape: a fast-spinning head, 4 lasers
# firing together, lines that drift slowly up and down. It also fills the FOV faster (21% of
# 0.5 deg cells at 0.1 s vs the proxy's 10%). Get it into third_party/livox_scan_mode/ and use
#   python scan_patterns.py --csv ../../../third_party/livox_scan_mode/mid360.csv
#
# Run (laptop, repo venv):  cd isaac_sim/unit3/lesson1_lidar_theory && python scan_patterns.py
# Output: figures/scan_patterns.png (scan_patterns_csv.png with --csv) and a coverage table.

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")                     # write PNG files; no window needed
import matplotlib.pyplot as plt

# --- 1. The two sensors (numbers from the datasheets; proxy shape for the Mid-360) ---------
VLP16 = dict(
    name="repetitive (VLP-16 class)",
    channels=16,                          # 16 lasers, one per elevation
    elev_deg=np.linspace(-15.0, 15.0, 16),   # fixed elevations, 2 deg apart
    rot_hz=10.0,                          # 10 revolutions per second
    az_step_deg=0.2,                      # one firing of all 16 lasers every 0.2 deg
)                                         # -> 1800 x 16 = 28,800 points per revolution (288k/s)

MID360 = dict(
    name="non-repetitive (Mid-360 class, proxy)",
    rate_hz=200_000,                      # points per second (Livox spec, first return)
    elev_min=-7.0, elev_max=52.0,         # vertical FOV (Livox spec)
    lasers=4,                             # proxy: 4 emitters sharing the point budget
    rot_hz=10.0,                          # proxy: head spins at the 10 Hz frame rate
    # Elevation oscillation frequency: an IRRATIONAL multiple of the rotation rate, so the
    # (azimuth, elevation) pair never comes back to the same value: that's what "non-repetitive"
    # means. A rational ratio (e.g. exactly 7x) would repeat every revolution.
    elev_hz=10.0 * (7 + (5 ** 0.5 - 1) / 2),   # 10 Hz x 7.618... (golden-ratio fraction)
)


def vlp16_pattern(duration_s, p=VLP16):
    """Azimuth/elevation (deg) and firing time (s) of every beam in duration_s."""
    n_fire = int(round(duration_s * p["rot_hz"] * 360.0 / p["az_step_deg"]))
    t_fire = np.arange(n_fire) / (p["rot_hz"] * 360.0 / p["az_step_deg"])   # firing times
    az = (t_fire * p["rot_hz"] * 360.0) % 360.0          # head angle when the lasers fire
    az = np.repeat(az, p["channels"])                     # all 16 lasers fire together
    el = np.tile(p["elev_deg"], n_fire)
    t = np.repeat(t_fire, p["channels"])
    return az, el, t


def mid360_proxy_pattern(duration_s, p=MID360):
    """Non-repetitive proxy: spinning head + elevation oscillation at an irrational ratio."""
    n = int(round(duration_s * p["rate_hz"]))
    t = np.arange(n) / p["rate_hz"]                       # one point every 5 microseconds
    k = np.arange(n) % p["lasers"]                        # which emitter fired this point
    az = (t * p["rot_hz"] * 360.0 + k * 360.0 / p["lasers"]) % 360.0
    mid = (p["elev_max"] + p["elev_min"]) / 2
    half = (p["elev_max"] - p["elev_min"]) / 2
    # arcsin(triangle wave) would be uniform; a plain sine is simpler and denser at the edges.
    phase = 2 * np.pi * (p["elev_hz"] * t + k / p["lasers"])
    el = mid + half * np.sin(phase)
    return az, el, t


_CSV_CACHE = {}


def load_csv(path, duration_s, rate_hz=MID360["rate_hz"]):
    """Load the measured Mid-360 pattern used by the Livox Gazebo plugins (scan_mode/mid360.csv).

    Format (checked Nov 18 against the file and the plugin source, livox_points_plugin.cpp):
      header "Time/s,Azimuth/deg,Zenith/deg", then 800,000 rows.
      column 0  despite its name, a ROW INDEX (1, 2, 3, ...), not seconds; the plugin doesn't
                use it either. Time is derived here: row i fires at (i - 1) / rate_hz.
      column 1  azimuth, degrees, 0..360 (decreasing: the head turns clockwise seen from above)
      column 2  ZENITH, degrees from straight up: elevation = 90 - zenith -> -7.2..52.2 deg
      Rows come in groups of 4 with nearly the same azimuth: 4 lasers fire together.
    800,000 rows at 200,000 points/s = 4 s of scanning; no 0.1 s frame repeats in it.
    """
    if path not in _CSV_CACHE:                            # 17 MB: read once, reuse
        data = np.loadtxt(path, delimiter=",", skiprows=1)
        t = (data[:, 0] - data[0, 0]) / rate_hz
        az, el = data[:, 1] % 360.0, 90.0 - data[:, 2]
        _CSV_CACHE[path] = (az, el, t)
        print(f"CSV {path}: {len(t)} rows = {t[-1]:.2f} s at {rate_hz} pts/s; "
              f"azimuth {az.min():.1f}..{az.max():.1f}, elevation {el.min():.1f}..{el.max():.1f} deg")
    az, el, t = _CSV_CACHE[path]
    keep = t < duration_s
    return az[keep], el[keep], t[keep]


# --- 2. Coverage: what fraction of the FOV has been hit at least once? -----------------------
def coverage(az, el, el_min, el_max, cell_deg):
    """Split the FOV into cell_deg x cell_deg cells; return the fraction hit at least once."""
    n_az = int(round(360.0 / cell_deg))
    n_el = int(round((el_max - el_min) / cell_deg))
    i = np.clip((az / cell_deg).astype(int), 0, n_az - 1)
    j = np.clip(((el - el_min) / cell_deg).astype(int), 0, n_el - 1)
    hit = np.zeros((n_el, n_az), dtype=bool)
    hit[j, i] = True
    return hit.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", help="measured Mid-360 pattern (mid360.csv) instead of the proxy")
    ap.add_argument("--cell", type=float, default=0.5, help="coverage cell size, deg")
    args = ap.parse_args()

    times = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]
    if args.csv:
        mid = lambda d: load_csv(args.csv, d)            # noqa: E731 (tiny adapter)
    else:
        mid = mid360_proxy_pattern

    # --- 3. Coverage table -------------------------------------------------------------
    print(f"\nFOV coverage, cells of {args.cell} x {args.cell} deg (fraction hit at least once)")
    print(f"{'integration':>12s} {'VLP-16 class':>14s} {'Mid-360 class':>15s}  points (VLP / Mid)")
    cov_v, cov_m = [], []
    for d in times:
        av, ev, _ = vlp16_pattern(d)
        am, em, _ = mid(d)
        cv = coverage(av, ev, -15.0, 15.0, args.cell)    # each sensor over ITS OWN FOV
        cm = coverage(am, em, MID360["elev_min"], MID360["elev_max"], args.cell)
        cov_v.append(cv)
        cov_m.append(cm)
        print(f"{d:10.2f} s {cv:13.1%} {cm:14.1%}   {len(av):>8d} / {len(am):d}")

    # --- 4. Figures ----------------------------------------------------------------------
    os.makedirs("figures", exist_ok=True)
    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    av, ev, _ = vlp16_pattern(0.1)
    ax[0, 0].scatter(av, ev, s=0.2, c="tab:blue")
    ax[0, 0].set(title="VLP-16 class, 0.1 s (one revolution): 16 fixed lines",
                 xlabel="azimuth (deg)", ylabel="elevation (deg)", ylim=(-10, 55))
    for d, a in ((0.1, ax[0, 1]), (1.0, ax[1, 0])):
        am, em, tm = mid(d)
        a.scatter(am, em, s=0.05, c=tm, cmap="viridis")  # colour = firing time
        a.set(title=f"Mid-360 class, {d} s (colour = firing time)",
              xlabel="azimuth (deg)", ylabel="elevation (deg)", ylim=(-10, 55))
    ax[1, 1].plot(times, cov_v, "o-", label=VLP16["name"])
    ax[1, 1].plot(times, cov_m, "s-", label=MID360["name"] if not args.csv else "Mid-360 (CSV)")
    ax[1, 1].set(xscale="log", xlabel="integration time (s)", ylabel="FOV coverage",
                 title=f"coverage of {args.cell} deg cells vs integration time", ylim=(0, 1.02))
    ax[1, 1].legend()
    ax[1, 1].grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig("figures/scan_patterns" + ("_csv" if args.csv else "") + ".png", dpi=110)
    print("\nwrote figures/scan_patterns" + ("_csv" if args.csv else "") + ".png")

    # --- 5. What to notice (answers are in the lesson notes) ------------------------------
    # 1. The repetitive sensor's coverage is FLAT after one revolution: integrating longer
    #    adds points on the same 16 lines, never between them. A thin horizontal bar between
    #    two lines (a feeder pan rim, a drinker line) can stay invisible forever while the
    #    robot stands still.
    # 2. The non-repetitive sensor starts sparser per 0.1 s frame (fewer points), but keeps
    #    filling the FOV. Standing still for 1 s, it "sees everything": good for mapping and
    #    for detecting small static things; but a single 0.1 s frame is irregular, which
    #    is why its drivers stamp EVERY point with its own time (motion distortion: see
    #    lidar_raycast_toy.py).
    #    Each sensor is measured over ITS OWN FOV: the Mid-360's is twice as tall (59 vs 30
    #    deg), so 76% of it is far more sky-and-floor than 27% of the VLP-16's.
    # 3. Change --cell to 0.2 and 2.0: coverage depends on the size of what you need to see.
    #    Ask "what is the smallest object at what distance?" (a chick 0.1 m at 5 m = 1.1 deg).


if __name__ == "__main__":
    main()
