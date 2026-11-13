"""
BARN-07 (first version): lighting and outdoor environment -> barns/<barn_id>/layers/lighting.usda

/World/Environment: daylight and the ground around the barn
    Sky     DomeLight: soft light from the whole sky (no texture yet)
    Sun     DistantLight: parallel sunlight, coming in at SUN_ELEVATION_DEG
    Ground  a flat quad GROUND_MARGIN_M beyond the walls, GROUND_Z below the floor top so
            the two never draw on top of each other (marked barn:allowBelowFloor)
/World/Barn/Lights: the barn lamps, PLACEHOLDERS until the second survey visit records the
    real ones (visit_02.md: type, count per row, rows at y, hang height). Every lamp carries
    barn:placeholder = true so they are easy to find and replace.

The shell is closed (walls and roof), so daylight only lights the outside; inside, the lamps
are the only light. Physics for the ground is in layers/physics.usda (build_physics.py).

Usage (venv active, from barn_usd/):
    python scripts/build_lighting.py barns/farm1_barn1
"""
import argparse
import math
import sys
from pathlib import Path

import yaml
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402

# Daylight
SKY_INTENSITY = 1000.0
SKY_COLOR = (0.70, 0.80, 1.00)        # bluish sky light
SUN_INTENSITY = 3000.0
SUN_COLOR = (1.00, 0.96, 0.90)        # slightly warm
SUN_ANGLE_DEG = 0.53                  # apparent size of the sun: sharp shadows
SUN_ELEVATION_DEG = 45.0              # above the horizon
SUN_AZIMUTH_DEG = 30.0                # from +x, counter-clockwise seen from above

# Outdoor ground
GROUND_MARGIN_M = 50.0                # beyond the walls on every side
GROUND_Z = -0.02                      # just below the floor top (Z=0): no z-fighting inside
GROUND_COLOR = (0.36, 0.33, 0.27)     # packed earth

# Barn lamps: PLACEHOLDER layout (typical poultry-house LED lamps), replace after visit_02
LAMP_ROW_OFFSETS_M = (-3.2, 3.2)      # rows either side of the ridge line
LAMP_SPACING_M = 6.0                  # along x
LAMP_END_GAP_M = 3.0                  # first/last lamp from the end walls
LAMP_HEIGHT_M = 2.3                   # hang height above the floor
LAMP_RADIUS_M = 0.06                  # bulb size
LAMP_INTENSITY = 30000.0              # RTX light units, not lumens
LAMP_EXPOSURE = 4.0                   # x 2**4 = 16: chosen by eye in Isaac Sim, camera inside the barn
LAMP_COLOR = (1.00, 0.93, 0.80)       # warm white


def add_daylight(stage):
    env = UsdGeom.Xform.Define(stage, "/World/Environment")

    sky = UsdLux.DomeLight.Define(stage, env.GetPath().AppendChild("Sky"))
    sky.CreateIntensityAttr(SKY_INTENSITY)
    sky.CreateColorAttr(Gf.Vec3f(*SKY_COLOR))

    # A DistantLight shines along its own -Z. Tilt it down from vertical by (90 - elevation),
    # then turn it to the azimuth: rotateXYZ applies X, then Y, then Z.
    sun = UsdLux.DistantLight.Define(stage, env.GetPath().AppendChild("Sun"))
    sun.CreateIntensityAttr(SUN_INTENSITY)
    sun.CreateColorAttr(Gf.Vec3f(*SUN_COLOR))
    sun.CreateAngleAttr(SUN_ANGLE_DEG)
    UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(0.0, 90.0 - SUN_ELEVATION_DEG, SUN_AZIMUTH_DEG))
    return env


def add_ground(stage, env, x0, x1, y0, y1):
    gx0, gx1 = x0 - GROUND_MARGIN_M, x1 + GROUND_MARGIN_M
    gy0, gy1 = y0 - GROUND_MARGIN_M, y1 + GROUND_MARGIN_M
    ground = UsdGeom.Mesh.Define(stage, env.GetPath().AppendChild("Ground"))
    pts = [(gx0, gy0, GROUND_Z), (gx1, gy0, GROUND_Z), (gx1, gy1, GROUND_Z), (gx0, gy1, GROUND_Z)]
    ground.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])          # counter-clockwise: faces +Z
    ground.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    ground.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(ground.GetPointsAttr().Get()))
    ground.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_COLOR)])
    # The validator's floor check: nothing below Z=-0.01 unless explicitly allowed.
    ground.GetPrim().CreateAttribute("barn:allowBelowFloor", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
    return ground


def add_lamps(stage, x0, x1, ridge_y):
    lights = UsdGeom.Xform.Define(stage, "/World/Barn/Lights")
    lights.GetPrim().SetCustomDataByKey("barn:source", "placeholder until survey/visit_02.md (lights)")
    n = int(math.floor((x1 - x0 - 2 * LAMP_END_GAP_M) / LAMP_SPACING_M)) + 1
    count = 0
    for r, dy in enumerate(LAMP_ROW_OFFSETS_M):
        for i in range(n):
            x = x0 + LAMP_END_GAP_M + i * LAMP_SPACING_M
            lamp = UsdLux.SphereLight.Define(stage, lights.GetPath().AppendChild(f"Lamp_R{r}_{i:02d}"))
            lamp.CreateRadiusAttr(LAMP_RADIUS_M)
            lamp.CreateIntensityAttr(LAMP_INTENSITY)
            lamp.CreateExposureAttr(LAMP_EXPOSURE)          # brightness x 2**exposure (photographic stops)
            lamp.CreateColorAttr(Gf.Vec3f(*LAMP_COLOR))
            UsdGeom.XformCommonAPI(lamp).SetTranslate(Gf.Vec3d(x, ridge_y + dy, LAMP_HEIGHT_M))
            lamp.GetPrim().CreateAttribute("barn:placeholder", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
            count += 1
    return count


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    args = ap.parse_args()

    by = yaml.safe_load((args.barn_dir / "survey" / "barn.yaml").read_text())
    e = by["extent"]
    x0, x1, y0, y1 = e["x_min_m"], e["x_max_m"], e["y_min_m"], e["y_max_m"]
    ridge_y = (y0 + y1) / 2                      # same centre line as build_shell.py

    out = args.barn_dir / "layers" / "lighting.usda"
    if out.exists():
        out.unlink()                             # generated layer: rebuilt from scratch
    stage = usd_utils.new_layer(out)
    env = add_daylight(stage)
    add_ground(stage, env, x0, x1, y0, y1)
    n_lamps = add_lamps(stage, x0, x1, ridge_y)
    stage.GetRootLayer().Save()
    print(f"-> {out}  (sky + sun, ground {x1 - x0 + 2 * GROUND_MARGIN_M:.0f} x "
          f"{y1 - y0 + 2 * GROUND_MARGIN_M:.0f} m, {n_lamps} PLACEHOLDER lamps at {LAMP_HEIGHT_M} m)")


if __name__ == "__main__":
    main()
