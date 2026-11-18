# Unit 3, Lesson 4 (GPU capture, laptop analysis): how realistic is the simulated lidar?
#
# Idea: capture the SAME frame twice in Isaac Sim, with the same pattern and pose:
#   ideal run   noise and dropouts OFF  -> z*  (what the ray caster says: the "map")
#   real-ish    noise, materials, reflectivity-dependent range ON  -> z
# Then fit Lesson 1's beam model to the (z, z*) pairs. The learned weights and sigma ARE the
# simulator's sensor model, measured. Compare them to:
#   - the datasheet (sigma 2 cm at 10 m)
#   - later, a REAL Mid-360 log in the real barn (Unit 4): where they differ, fix the sim.
#
# Topics for the lesson (Isaac side, [UNVERIFIED] names): range noise settings on the
# OmniLidar profile; materials and non-visual material properties (reflectivity) on barn
# surfaces; intensity; MotionBVH (motion during the scan: Unit 2 Lesson 5 warning); multiple
# returns; dust/feathers (can't be ray traced cheaply: model as p_short / p_rand on purpose).
#
# Analysis (written; runs on the laptop once the two .npz files exist):
#   python fit_sim_beam_model.py ideal.npz noisy.npz

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lesson1_lidar_theory"))
import numpy as np                                    # noqa: E402
from beam_model import Z_MAX, learn                   # noqa: E402  (Lesson 1)


def main(ideal_path, noisy_path):
    ideal, noisy = np.load(ideal_path), np.load(noisy_path)
    zs, z = ideal["range"], noisy["range"]            # same beams, same order [CHECK on capture]
    if len(zs) != len(z):
        raise SystemExit("frames differ in length: capture with a fixed pattern and pose")
    z = np.where(np.isfinite(z) & (z > 0), z, Z_MAX)  # no return -> z_max, as the model expects
    ok = np.isfinite(zs) & (zs > 0) & (zs < Z_MAX)    # beams that hit something in the ideal run
    est = learn(z[ok], zs[ok])
    print(f"{ok.sum()} beams")
    for k, v in est.items():
        print(f"  {k:8s} {v:.4f}")
    # Next: the same per range bin (0-5, 5-10, 10-20 m): is sigma constant, as the simple
    # model assumes, or does it grow with range like real sensors?


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: python fit_sim_beam_model.py ideal.npz noisy.npz")
    main(sys.argv[1], sys.argv[2])
