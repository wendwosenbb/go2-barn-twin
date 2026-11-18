# Unit 3, Lesson 3 (GPU): a Livox Mid-360 twin, a NON-REPETITIVE lidar Isaac Sim doesn't ship.
#
# There is no official Mid-360 profile in Isaac Sim; you build one by giving the RTX lidar a
# custom emitter pattern. That's the real job of a sensor-simulation engineer: turn a datasheet
# (+ a measured pattern) into a validated model.
#
# Target (Livox official specs):
#   360 deg horizontal x -7..52 deg vertical; 200,000 points/s (first return); 10 Hz frames
#   range 40 m @ 10% reflectivity (70 m @ 80%); minimum range 0.1 m
#   range accuracy <= 2 cm @ 10 m; angular accuracy < 0.15 deg
#   non-repetitive pattern: ~0.1 s coverage similar to a 32-line spinning lidar, growing with time
#   built-in IMU (ICM40609, 6-axis): publish it too (Lesson 5)
#   driver output (livox_ros_driver2): PointCloud2 or CustomMsg with per-point offset_time,
#   reflectivity, tag, line -> the twin should publish the SAME fields (FAST-LIO reads them)
#
# THE RTX RECIPE (NVIDIA staff, github.com/isaac-sim/IsaacSim/discussions/685, Jun-Aug 2026,
# for Isaac Sim 5.1/6.0; check every name in 6.1):
#   - The RTX lidar supports non-repetitive patterns natively through EMITTER STATES: an array
#     of states, each a full list of emitters with REQUIRED fields azimuthDeg, elevationDeg,
#     fireTimeNs, channelId (optional: bank, rangeId, offsets).
#   - emitterStateCount = N states; stateResolutionStep = 1 -> frame 0 uses state 0, frame 1
#     state 1, ..., then it loops. So a measured pattern of N frames plays back exactly.
#   - Settings on the prim (namespace omni:sensor:Core:): scanType SOLID_STATE, rayType
#     IDEALIZED, numberOfEmitters = emitters per state, tickRate 10 (from 6.0, "multi-tick
#     rendering" really limits the sensor to 10 Hz; before 6.0 every RTX sensor rendered every
#     sim frame -> 6x too many points at 60 Hz).
#   - BUDGET: about 5 MiB per lidar prim ~= emitterStateCount x numberOfEmitters x 16 bytes.
#     Over budget the error goes to an internal RTX log (NOT the console) and the sensor is
#     torn down and rebuilt every frame: it looks like severe lag. -> split across prims.
#   - The forum user's working setup: 4 prims (one per laser: the Mid-360 fires 4 at once),
#     5000 emitters x 25 states each. Problems seen on 5.1: crashes with ROS 2 helpers
#     (staff: known GPU-buffer race; flag --/app/sensors/nv/lidar/outputBufferOnGPU=false),
#     partial frames (ROS2 RTX lidar helper: fullScan = True, point_cloud type only).
#
# Run:
#   laptop:  python mid360_twin.py ../../../third_party/livox_scan_mode/mid360.csv
#            (checks the pattern against the datasheet, splits it into emitter states, checks
#             the budget, writes mid360_emitter_states.npz for the VM step)
#   VM:      docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/mid360_twin.py --vm
#
# Status: laptop part WRITTEN and run; VM part OUTLINE ([UNVERIFIED] for 6.1).

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lesson1_lidar_theory"))
from scan_patterns import coverage, load_csv          # noqa: E402  (Lesson 1; format notes there)

RATE_HZ = 200_000          # points per second (spec) -> time of row i = i / RATE_HZ
FRAME_S = 0.1              # 10 Hz frames
LASERS = 4                 # the measured file fires 4 lasers together (rows in groups of 4)
BYTES_PER_EMITTER = 16     # 4 required fields (staff's estimate)
BUDGET = 5 * 1024 * 1024   # bytes per lidar prim (staff: RTX Hydra render bridge limit)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../../third_party/livox_scan_mode/mid360_emitter_states.npz")   # derived from
#   third-party data: kept in git-ignored third_party/, not committed


def build_emitter_states(az, el, t, n_frames):
    """Split the pattern into per-laser prims x per-frame states (the forum's layout).

    Returns arrays shaped (LASERS, n_frames, emitters_per_state) for azimuth, elevation,
    fire time within the frame (ns) and channel id. Row i belongs to laser i % LASERS.
    """
    per_frame = int(round(FRAME_S * RATE_HZ))             # 20,000 points per frame
    per_state = per_frame // LASERS                       # 5,000 emitters per laser per frame
    need = n_frames * per_frame
    if len(az) < need:
        raise SystemExit(f"pattern has {len(az)} points, need {need} for {n_frames} frames")
    shape = (n_frames, per_state, LASERS)                 # frame, firing, laser
    a = az[:need].reshape(shape).transpose(2, 0, 1)       # -> laser, frame, firing
    e = el[:need].reshape(shape).transpose(2, 0, 1)
    # Fire time within its frame, in whole ns from the row index (5,000 ns per point at
    # 200k/s): floating-point t % 0.1 can round up to 0.1 s, which is the NEXT frame.
    ns_per_point = int(round(1e9 / RATE_HZ))
    tf = ((np.arange(need) % per_frame) * ns_per_point).reshape(shape).transpose(2, 0, 1)
    ch = np.broadcast_to(np.arange(LASERS)[:, None, None], a.shape)
    return a, e, tf, ch


def main_laptop(path):
    az, el, t = load_csv(path, 1e9, rate_hz=RATE_HZ)      # whole file
    n_frames = len(az) // int(round(FRAME_S * RATE_HZ))   # full frames (40 in 800,000 rows);
    # counting from rows, not from t[-1] (3.99999 s would give 39)
    print(f"\n1. Pattern vs datasheet")
    print(f"   points per {FRAME_S} s frame: {int(FRAME_S * RATE_HZ)}  (spec: 200,000/s)")
    print(f"   elevation {el.min():.2f}..{el.max():.2f} deg  (spec: -7..52)")
    for d in (0.1, 1.0):
        k = t < d
        print(f"   coverage of 0.5 deg cells after {d} s: {coverage(az[k], el[k], -7.3, 52.3, 0.5):.1%}")

    a, e, tf, ch = build_emitter_states(az, el, t, n_frames)
    per_state = a.shape[2]
    print(f"\n2. Emitter states: {LASERS} prims (one per laser) x {n_frames} states x {per_state} emitters")
    # Check the split: each laser (row slot k of every 4-row group) has its OWN elevation band.
    # The 4 lasers fire together but NOT at one azimuth (typically ~4 deg apart), as the forum
    # user also found: "4 beams at different zenith and azimuth firing at the same time".
    for k in range(LASERS):
        print(f"   laser {k}: elevation {e[k].min():6.2f}..{e[k].max():6.2f} deg")
    g = np.ptp(a[:, :, :], axis=0)
    g = np.minimum(g, 360 - g)
    print(f"   azimuth spread between the 4 lasers of one firing: median {np.median(g):.1f} deg")
    print(f"   fire times within a frame: {tf.min():.0f}..{tf.max():.0f} ns")

    print(f"\n3. Budget per prim (limit {BUDGET / 2**20:.0f} MiB ~ states x emitters x {BYTES_PER_EMITTER} B)")
    for states in (25, n_frames, 65, 100):
        size = states * per_state * BYTES_PER_EMITTER
        print(f"   {states:3d} states: {size / 2**20:5.2f} MiB  {'ok' if size <= BUDGET else 'OVER: silent rebuild every frame'}")
    max_states = BUDGET // (per_state * BYTES_PER_EMITTER)
    print(f"   -> at most {max_states} states per prim = {max_states * FRAME_S:.1f} s before the pattern loops")
    print("   A single prim with all 20,000 emitters/frame would allow only "
          f"{BUDGET // (per_state * LASERS * BYTES_PER_EMITTER)} states: that's why the split by laser.")

    np.savez(OUT, azimuthDeg=a.astype(np.float32), elevationDeg=e.astype(np.float32),
             fireTimeNs=tf.astype(np.uint32), channelId=ch.astype(np.uint32))
    print(f"\nwrote {os.path.normpath(OUT)} (copy it to the VM with this script)")


# --- VM part (OUTLINE, [UNVERIFIED] for 6.1) ------------------------------------------------
# 1. SimulationApp headless (consider the flag --/app/sensors/nv/lidar/outputBufferOnGPU=false
#    if it crashes with ROS 2 helpers, as on 5.1).
# 2. Open the barn, a static stand at the Go2 mount pose (as in Lesson 2).
# 3. For each laser k in 0..3: an OmniLidar prim "Mid360_k" under the stand, with
#    omni:sensor:Core:scanType SOLID_STATE, rayType IDEALIZED, numberOfEmitters = 5000,
#    emitterStateCount = 40, stateResolutionStep = 1, tickRate 10, ranges 0.1..40 m, and its
#    emitter states from mid360_emitter_states.npz[ :, k ]. FIND in 6.1: the exact attribute
#    names and array layout for emitter states (print an OmniLidar prim's attributes, and
#    look at a shipped SOLID_STATE profile for the pattern to copy).
# 4. Play; capture frames (Lesson 2's annotator); check per frame: 20,000 points total
#    (4 x 5,000), elevation -7..52, per-point time 0..0.1 s, and coverage after 1 s ~84%
#    (the same numbers this script printed for the pattern itself). Any difference = the
#    twin isn't playing the pattern as intended.
# 5. Publish: one PointCloud2 merging the 4 prims, in the Mid-360 frame, with per-point time
#    (ROS 2 RTX lidar helper: point_cloud type, fullScan True), like livox_ros_driver2.


if __name__ == "__main__":
    if "--vm" in sys.argv:
        raise SystemExit("VM part not written yet: see the outline at the end of this file")
    main_laptop(sys.argv[1] if len(sys.argv) > 1 else "../../../third_party/livox_scan_mode/mid360.csv")
