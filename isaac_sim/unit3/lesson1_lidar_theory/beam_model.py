# Unit 3, Lesson 1 (theory, no GPU): the BEAM MODEL of a range sensor.
# Companion to Probabilistic Robotics (Thrun, Burgard, Fox), chapter 6.2-6.3.
#
# A simulator's ray caster gives the TRUE range z* to the first surface along a beam. A real
# sensor returns a measured z that differs from z* in four typical ways. The beam model mixes
# four densities, one per way:
#
#   p_hit    N(z*, sigma^2), cut to [0, z_max]  the beam hit the expected surface; small noise
#   p_short  exponential decay, only z < z*     something UNEXPECTED was in between
#                                               (in the barn: a chicken, a feather, dust)
#   p_max    a spike at z = z_max               no return at all (black or wet surfaces, glass,
#                                               mirror-like metal, out of range): reported as max
#   p_rand   uniform on [0, z_max]              unexplained garbage (crosstalk, sunlight, dust)
#
#   p(z | z*) = w_hit p_hit + w_short p_short + w_max p_max + w_rand p_rand   (weights sum to 1)
#
# The script: (1) plots the density, (2) samples a "real" log from it, (3) RECOVERS the
# parameters from that log with the book's learning algorithm (Table 6.2). Step 3 is exactly
# Unit 4's "calibrate sim parameters from real logs": fit to real data, then make the sim
# produce the same distribution.
#
# Run (laptop, repo venv):  cd isaac_sim/unit3/lesson1_lidar_theory && python beam_model.py

import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from math import erf, sqrt

erf_v = np.vectorize(erf)      # math.erf takes one number; this applies it to arrays

# --- 1. Parameters: a Mid-360-like sensor in a barn (true values for this experiment) ------
Z_MAX = 40.0      # m, Livox spec range at 10% reflectivity (simplification: one z_max)
TRUE = dict(
    w_hit=0.80,   # most beams hit what the map says
    w_short=0.10, # birds and dust in front of the wall: frequent in a poultry house
    w_max=0.06,   # no return: e.g. black plastic, open doors/fans, wet surfaces
    w_rand=0.04,  # random returns
    sigma=0.05,   # m. Datasheet: <= 2 cm at 10 m (lab, white target). Barn reality is worse:
                  # dust, incidence angle, mixed pixels at edges -> use more, then MEASURE it
    lam=0.5,      # 1/m: rate of the "short" exponential (larger = obstacles closer to sensor)
)


def p_hit(z, zs, sigma):
    """Gaussian around the true range, renormalised to [0, Z_MAX] (the eta of the book)."""
    cdf = lambda x: 0.5 * (1 + erf_v((x - zs) / (sigma * sqrt(2))))  # noqa: E731
    eta = 1.0 / (cdf(Z_MAX) - cdf(0.0))
    g = np.exp(-0.5 * ((z - zs) / sigma) ** 2) / (sigma * sqrt(2 * np.pi))
    return np.where((z >= 0) & (z <= Z_MAX), eta * g, 0.0)


def p_short(z, zs, lam):
    """Exponential for z < z*: an unexpected object is more likely close to the sensor."""
    eta = 1.0 / (1.0 - np.exp(-lam * zs))
    return np.where((z >= 0) & (z < zs), eta * lam * np.exp(-lam * z), 0.0)


def p_max(z, width=0.01):
    """Point mass at Z_MAX, drawn as a narrow box (width m) so it can be plotted and fitted."""
    return np.where(z >= Z_MAX - width, 1.0 / width, 0.0)


def p_rand(z):
    return np.where((z >= 0) & (z < Z_MAX), 1.0 / Z_MAX, 0.0)


def density(z, zs, p):
    return (p["w_hit"] * p_hit(z, zs, p["sigma"]) + p["w_short"] * p_short(z, zs, p["lam"])
            + p["w_max"] * p_max(z) + p["w_rand"] * p_rand(z))


def sample(zs, p, n, rng):
    """Draw n readings for true ranges zs (array): pick a cause, then draw from it."""
    cause = rng.choice(4, size=n, p=[p["w_hit"], p["w_short"], p["w_max"], p["w_rand"]])
    z = np.empty(n)
    m = cause == 0
    z[m] = np.clip(rng.normal(zs[m], p["sigma"]), 0, Z_MAX)
    m = cause == 1                        # inverse-CDF sampling of the truncated exponential
    u = rng.random(m.sum())
    z[m] = -np.log(1 - u * (1 - np.exp(-p["lam"] * zs[m]))) / p["lam"]
    m = cause == 2
    z[m] = Z_MAX
    m = cause == 3
    z[m] = rng.uniform(0, Z_MAX, m.sum())
    return z


def learn(z, zs, iters=30):
    """Probabilistic Robotics Table 6.2: estimate weights, sigma, lambda from (z, z*) pairs.

    Expectation-maximisation: for each reading, how much does each cause explain it
    (responsibility e_*), then re-estimate the parameters from those responsibilities.
    """
    p = dict(w_hit=0.25, w_short=0.25, w_max=0.25, w_rand=0.25, sigma=0.5, lam=1.0)  # rough start
    for _ in range(iters):
        a = np.stack([p["w_hit"] * p_hit(z, zs, p["sigma"]),
                      p["w_short"] * p_short(z, zs, p["lam"]),
                      p["w_max"] * p_max(z),
                      p["w_rand"] * p_rand(z)])
        e = a / np.maximum(a.sum(axis=0), 1e-300)         # responsibilities, columns sum to 1
        n = len(z)
        p["w_hit"], p["w_short"], p["w_max"], p["w_rand"] = (e.sum(axis=1) / n).tolist()
        p["sigma"] = float(np.sqrt((e[0] * (z - zs) ** 2).sum() / e[0].sum()))
        p["lam"] = float(e[1].sum() / (e[1] * z).sum())
    return p


def main():
    rng = np.random.default_rng(0)        # fixed seed: same "log" every run (reproducible)
    os.makedirs("figures", exist_ok=True)

    # --- 2. The density for one beam whose true range is 5 m ------------------------------
    zs1 = 5.0
    zz = np.linspace(0, Z_MAX, 40001)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    ax[0].plot(zz, density(zz, zs1, TRUE))
    ax[0].set(title=f"p(z | z* = {zs1} m), full range", xlabel="measured z (m)", yscale="log",
              ylim=(1e-4, 20))
    near = (zz > zs1 - 0.3) & (zz < zs1 + 0.3)
    ax[1].plot(zz[near], density(zz[near], zs1, TRUE))
    ax[1].set(title="zoom around z*: the hit peak (sigma)", xlabel="measured z (m)")

    # --- 3. A synthetic "real log": 20,000 beams at true ranges 1..20 m --------------------
    zs = rng.uniform(1.0, 20.0, 20000)    # ranges the ray caster would report (the map)
    z = sample(zs, TRUE, len(zs), rng)    # what the "real" sensor reported
    err = z - zs
    ax[2].hist(err[(err > -6) & (err < 2)], bins=200, density=True)
    ax[2].set(title="measured - true range (log of 20,000 beams)", xlabel="error (m)",
              yscale="log")
    fig.tight_layout()
    fig.savefig("figures/beam_model.png", dpi=110)
    print("wrote figures/beam_model.png")

    # --- 4. Calibrate: recover the parameters from the log --------------------------------
    est = learn(z, zs)
    print(f"\n{'parameter':10s} {'true':>8s} {'learned':>9s}")
    for k in TRUE:
        print(f"{k:10s} {TRUE[k]:8.3f} {est[k]:9.3f}")

    # --- 5. What to notice --------------------------------------------------------------
    # 1. An IDEAL simulator gives only the hit peak with sigma = 0 (a single line at z*).
    #    Every other term is something the sim must ADD (noise model) or produce by having
    #    the cause in the scene (birds -> real short readings, dark materials -> real misses).
    #    Prefer causes in the scene over added randomness where they matter: added "short"
    #    readings don't move with the birds.
    # 2. The learned weights are a fingerprint of an environment. Fit them on a real barn log
    #    and on a sim log of the same route: where they differ, the twin is wrong (Unit 4).
    # 3. lambda comes out ~0.62, not 0.5: Table 6.2's update (sum e / sum e*z) is the
    #    estimate for an UNtruncated exponential, but p_short is cut off at z*, so long short
    #    readings are missing and lambda is overestimated. The weights and sigma are fine.
    #    Lesson: check a published estimator against data with KNOWN parameters before
    #    trusting it on real logs (exactly what this script does).
    # 4. Try sigma 0.02 (datasheet) vs 0.05 here: a SLAM system tuned on the clean value
    #    can fail on the real one. Simulate the conditions you'll meet, not the datasheet's.


if __name__ == "__main__":
    main()
