"""
SIM-01 (first part): physics for the barn -> barns/<barn_id>/layers/physics.usda

Only barn_sim.usda includes this layer; barn.usda (rendering) stays physics-free.

/World/PhysicsScene      the one physics scene: gravity -Z 9.81, 200 steps per second
                         (Go2 runs at Isaac Lab's rate; see Unit 2 Lesson 3, B8)
over /World/Barn/Shell/* static colliders on the floor slab and the walls (triangle mesh:
                         exact shape, fine for things that never move). The roof is left
                         out: nothing reaches it.
/World/Environment/GroundCollider
                         infinite collision plane at the outdoor ground height, guide
                         purpose (not rendered; the visible ground is in lighting.usda)

Not yet: colliders for the equipment (feeder pans, drinker lines). Their phys.usda files
exist per asset (make_asset.py) and get pulled in here later.

PhysxSceneAPI is applied by name, so this runs with plain OpenUSD (no PhysxSchema needed).

Usage (venv active, from barn_usd/):
    python scripts/build_physics.py barns/farm1_barn1
"""
import argparse
import sys
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

sys.path.insert(0, str(Path(__file__).parent))
import usd_utils  # noqa: E402
from build_lighting import GROUND_Z  # noqa: E402  keep the collider at the visible ground

STEPS_PER_SECOND = 200
SHELL = "/World/Barn/Shell"
COLLIDE = ["Floor", "Wall_YNeg", "Wall_YPos", "Wall_XNeg", "Wall_XPos"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("barn_dir", type=Path)
    args = ap.parse_args()

    out = args.barn_dir / "layers" / "physics.usda"
    if out.exists():
        out.unlink()                                       # generated layer: rebuilt from scratch
    stage = usd_utils.new_layer(out)

    # The physics scene
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    prim = scene.GetPrim()
    prim.AddAppliedSchema("PhysxSceneAPI")                 # PhysX settings, applied by name
    prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(STEPS_PER_SECOND)

    # Static colliders on the shell: 'over' = add to prims defined in shell.usda
    for name in COLLIDE:
        p = stage.OverridePrim(f"{SHELL}/{name}")
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(UsdPhysics.Tokens.none)  # exact triangles

    # Outdoor ground collider
    plane = UsdGeom.Plane.Define(stage, "/World/Environment/GroundCollider")
    plane.CreateAxisAttr(UsdGeom.Tokens.z)
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.XformCommonAPI(plane).SetTranslate(Gf.Vec3d(0.0, 0.0, GROUND_Z))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    stage.GetRootLayer().Save()
    print(f"-> {out}  (PhysicsScene {STEPS_PER_SECOND} Hz, colliders on {', '.join(COLLIDE)}, ground plane)")


if __name__ == "__main__":
    main()
