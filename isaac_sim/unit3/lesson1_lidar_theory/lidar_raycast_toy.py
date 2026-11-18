# Unit 3, Lesson 1 (theory, no GPU): a toy LIDAR SIMULATOR in the barn, in ~200 lines.
#
# Every lidar simulator, Isaac Sim's RTX lidar included, is these three steps:
#   1. SCAN PATTERN   which directions are fired, and WHEN          (scan_patterns.py)
#   2. RAY CASTING    the true range z* to the first surface along each beam
#                     (Isaac: on the GPU's RT cores, against the full USD scene)
#   3. SENSOR MODEL   turn z* into what the real sensor would report (beam_model.py),
#                     plus min/max range, no-return handling, timestamps
#
# Here the scene is the surveyed barn shell (barn.yaml: walls, gable ceiling, end wall), the 3
# surveyed bollards, and a flock of birds as spheres. The robot DRIVES and TURNS during the
# 0.1 s scan, which shows the effect every fast-moving lidar robot has to handle: MOTION
# DISTORTION. Each point is measured from a slightly different pose; if you pretend they were
# all measured at the end of the scan, straight walls bend. Fix ("deskewing"): stamp every
# point with its own time and transform it with the pose AT THAT TIME. That's why Livox
# drivers publish a per-point timestamp, and why FAST-LIO (the usual Mid-360 SLAM) needs it.
#
# Run (laptop, repo venv):  cd isaac_sim/unit3/lesson1_lidar_theory && python lidar_raycast_toy.py

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from beam_model import Z_MAX, sample                 # the sensor model (step 3)
from scan_patterns import load_csv, mid360_proxy_pattern   # the scan pattern (step 1)

# Measured Mid-360 pattern if it's there (git-ignored third_party/, see scan_patterns.py),
# otherwise the proxy. The report line says which one was used.
CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../../third_party/livox_scan_mode/mid360.csv")

# --- 1. The scene: surveyed barn shell (survey/barn.yaml, barn frame: x down the barn) -----
X_MIN, X_MAX = -0.95, 154.35       # end walls
Y_MIN, Y_MAX = -8.02, 4.78         # long walls (12.8 m apart)
RIDGE_Y, RIDGE_Z, WALL_Z = -1.6, 3.5, 2.65   # gable ceiling: 3.5 m at the ridge, 2.65 at walls
K = (RIDGE_Z - WALL_Z) / (Y_MAX - RIDGE_Y)   # ceiling slope (m per m), same on both sides
BOLLARDS = [(-0.499, -3.462), (-0.419, 0.386), (1.972, -3.434)]   # survey equipment.csv
BOLLARD_R, BOLLARD_H = 0.08, 1.0   # ASSUMED size (not in the survey)
BIRD_R = 0.12                      # m, a sphere roughly the size of a broiler's body

# --- 2. The robot and its motion during one scan -------------------------------------------
SENSOR_H = 0.40                    # m above the floor: roughly a Mid-360 on a Go2's back
START = (3.0, 0.31)                # x, y at the start of the scan (Lesson 6's aisle centre)
V = 1.0                            # m/s forward (a brisk Go2 trot)
W = 0.5                            # rad/s turning left
SCAN_S = 0.1                       # one 10 Hz frame

# Sensor model for this toy: datasheet-like noise, few "short"/random returns, because birds
# are IN the scene (real short readings) instead of being faked by p_short.
MODEL = dict(w_hit=0.95, w_short=0.01, w_max=0.02, w_rand=0.02, sigma=0.02, lam=0.5)
R_MIN = 0.1                        # m, Livox minimum range: closer returns are dropped


def pose(t):
    """Sensor position (3,) arrays and yaw at times t: constant speed V and turn rate W."""
    yaw = W * t
    x = START[0] + V / W * np.sin(yaw)
    y = START[1] + V / W * (1 - np.cos(yaw))
    return np.stack([x, y, np.full_like(t, SENSOR_H)], axis=1), yaw


def rotz(v, yaw):
    """Rotate vectors v (N,3) about z by yaw (N,)."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]], axis=1)


def cast(o, d, birds):
    """True range along each ray (origins o, unit directions d; both (N,3)). inf = no hit."""
    best = np.full(len(o), np.inf)

    def keep(t, ok):                               # take t where valid and nearer than before
        np.minimum(best, np.where(ok & (t > 1e-9), t, np.inf), out=best)

    with np.errstate(divide="ignore", invalid="ignore"):
        keep(-o[:, 2] / d[:, 2], d[:, 2] < 0)                    # floor z = 0
        keep((Y_MIN - o[:, 1]) / d[:, 1], d[:, 1] < 0)           # right long wall
        keep((Y_MAX - o[:, 1]) / d[:, 1], d[:, 1] > 0)           # left long wall
        keep((X_MIN - o[:, 0]) / d[:, 0], d[:, 0] < 0)           # end wall behind the start
        keep((X_MAX - o[:, 0]) / d[:, 0], d[:, 0] > 0)           # far end wall
        for side in (+1, -1):                                    # the two ceiling halves:
            n = np.array([0.0, K * side, 1.0])                   # z + K*side*y = const
            c = RIDGE_Z + K * side * RIDGE_Y
            t = (c - o @ n) / (d @ n)
            yh = o[:, 1] + t * d[:, 1]
            keep(t, (yh >= RIDGE_Y) if side > 0 else (yh < RIDGE_Y))
        for cx, cy in BOLLARDS:                                  # vertical cylinders
            ox, oy = o[:, 0] - cx, o[:, 1] - cy
            a = d[:, 0] ** 2 + d[:, 1] ** 2
            b = 2 * (d[:, 0] * ox + d[:, 1] * oy)
            disc = b * b - 4 * a * (ox * ox + oy * oy - BOLLARD_R ** 2)
            t = (-b - np.sqrt(disc)) / (2 * a)
            zh = o[:, 2] + t * d[:, 2]
            keep(t, (disc >= 0) & (zh >= 0) & (zh <= BOLLARD_H))
        for c in birds:                                          # spheres
            oc = o - c
            b = 2 * np.einsum("ij,ij->i", d, oc)
            disc = b * b - 4 * (np.einsum("ij,ij->i", oc, oc) - BIRD_R ** 2)
            keep((-b - np.sqrt(disc)) / 2, disc >= 0)
    return best


def main():
    rng = np.random.default_rng(1)
    os.makedirs("figures", exist_ok=True)
    # Birds: 150 on the floor near the robot (x 0..20), not inside the robot's footprint
    birds = np.column_stack([rng.uniform(0, 20, 400), rng.uniform(Y_MIN + 0.3, Y_MAX - 0.3, 400),
                             np.full(400, BIRD_R)])
    birds = birds[np.hypot(birds[:, 0] - START[0], birds[:, 1] - START[1]) > 1.0][:150]

    # --- 3. Step 1: pattern (sensor frame) and per-point times ---------------------------
    if os.path.exists(CSV):
        az, el, t = load_csv(CSV, SCAN_S)
        print("pattern: measured Mid-360 (mid360.csv)")
    else:
        az, el, t = mid360_proxy_pattern(SCAN_S)
        print("pattern: PROXY (mid360.csv not found): shape is not the real Mid-360's")
    a, e = np.radians(az), np.radians(el)
    d_local = np.column_stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])

    # --- 4. Step 2: ray casting from the pose at EACH point's firing time -----------------
    o, yaw = pose(t)
    d_world = rotz(d_local, yaw)
    z_true = cast(o, d_world, birds)

    # --- 5. Step 3: sensor model ----------------------------------------------------------
    hit = np.isfinite(z_true) & (z_true <= Z_MAX)
    z = np.full(len(t), Z_MAX)                           # no hit -> no return (reported as max)
    z[hit] = sample(z_true[hit], MODEL, hit.sum(), rng)
    valid = (z >= R_MIN) & (z < Z_MAX)                   # what the driver actually publishes
    p_local = d_local[valid] * z[valid, None]            # point in the sensor frame at ITS time
    tv = t[valid]
    print(f"{len(t)} beams in {SCAN_S} s; {hit.sum()} hit something; "
          f"{valid.sum()} published (min range {R_MIN} m, no-returns dropped)")

    # --- 6. Motion distortion: naive vs deskewed -------------------------------------------
    o_end, yaw_end = pose(np.array([SCAN_S]))
    naive = rotz(p_local, np.full(len(tv), yaw_end[0])) + o_end      # all at the END pose
    o_i, yaw_i = pose(tv)
    deskewed = rotz(p_local, yaw_i) + o_i                            # each at ITS pose
    err = np.linalg.norm(naive - deskewed, axis=1)
    rng_m = np.linalg.norm(p_local, axis=1)
    print(f"motion distortion (v {V} m/s, w {W} rad/s, {SCAN_S} s): "
          f"mean {err.mean() * 100:.1f} cm, 95% {np.percentile(err, 95) * 100:.1f} cm, "
          f"max {err.max() * 100:.1f} cm; at > 5 m range mean {err[rng_m > 5].mean() * 100:.1f} cm")

    # --- 7. Figures -------------------------------------------------------------------------
    sl = (deskewed[:, 2] > 0.2) & (deskewed[:, 2] < 1.5)             # a horizontal slice
    fig, ax = plt.subplots(1, 2, figsize=(15, 6))
    ax[0].scatter(deskewed[sl, 0], deskewed[sl, 1], s=1, c="tab:blue", label="deskewed (per-point pose)")
    ax[0].scatter(naive[sl, 0], naive[sl, 1], s=1, c="tab:red", alpha=0.5, label="naive (end pose)")
    tr, _ = pose(np.linspace(0, SCAN_S, 20))
    ax[0].plot(tr[:, 0], tr[:, 1], "k-", lw=3, label="robot path during the scan")
    ax[0].axhline(Y_MIN, c="gray", lw=0.5)
    ax[0].axhline(Y_MAX, c="gray", lw=0.5)
    ax[0].axvline(X_MIN, c="gray", lw=0.5)
    ax[0].set(xlim=(-2, 25), ylim=(-9, 6), aspect="equal", xlabel="x (m)", ylabel="y (m)",
              title="slice z 0.2-1.5 m: walls, bollards, birds (gray = true walls)")
    ax[0].legend(loc="upper right", markerscale=6)
    ax[1].scatter(rng_m, err * 100, s=0.5, c=tv, cmap="viridis")
    ax[1].set(xlabel="range (m)", ylabel="naive error (cm)",
              title="distortion grows with range (rotation) and with firing time (colour)")
    fig.tight_layout()
    fig.savefig("figures/lidar_raycast_toy.png", dpi=110)
    print("wrote figures/lidar_raycast_toy.png")

    # --- 8. What to notice --------------------------------------------------------------
    # 1. Points fired EARLY in the scan (dark) have the biggest error: the robot moved most
    #    between their firing and the end pose. Error = translation (V * dt) + rotation
    #    (W * dt * range): at 10 m, 0.05 rad is 50 cm. Turning hurts far more than driving.
    # 2. A walking Go2 also pitches and rolls every step: the same effect in 3D, at the gait
    #    frequency. This is why legged-robot lidar SLAM fuses the IMU (the Mid-360 has one).
    # 3. In Isaac Sim the RTX lidar casts each beam at its own firing time while the scene
    #    moves, IF motion is enabled (Lesson 5 warned "MotionBVH ... not enabled": then every
    #    beam sees the scene frozen at one instant, and this distortion is MISSING from the sim).
    #    A twin that hides distortion makes SLAM look better in sim than it will be on the robot.
    # 4. Set W = 0 and V = 0: naive == deskewed. Set W = 1.0: watch the walls bend.


if __name__ == "__main__":
    main()
