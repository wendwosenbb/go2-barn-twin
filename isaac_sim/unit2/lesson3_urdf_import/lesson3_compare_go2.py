# Unit 2, Lesson 3, B6: compare your imported Go2 with NVIDIA's Go2 assets.
#
# Run INSIDE Isaac Sim (Window > Script Editor > File > Open this file > Ctrl+Enter).
# Loads each robot under its own /World/<name> prim on a fresh stage (no physics is
# run; this only READS the authored values) and prints, per robot:
#   - how many moving (revolute) joints and fixed joints it has
#   - drive gains, torque limit (maxForce) and joint limits for one leg (FL)
#   - total mass, base mass
#   - the foot collision shape
#
# Values are in USD units: angles in degrees, stiffness N.m/deg, damping N.m.s/deg.
# Per-radian equivalents (what papers and Isaac Lab quote) are printed alongside.
import math
import sys

# Copying text OUT of Isaac Sim doesn't work over the stream, so everything printed
# below also goes to a file in the work folder (read it with ssh + cat).
REPORT = "/isaac-sim/work/lesson3_compare_go2.txt"


class Tee:
    """Send print() output to the Script Editor AND the report file."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for s in self.streams:
            s.write(text)

    def flush(self):
        for s in self.streams:
            s.flush()


_report = open(REPORT, "w")
_stdout = sys.stdout
sys.stdout = Tee(_stdout, _report)

import omni.usd
import omni.timeline
from pxr import Usd, UsdPhysics, UsdGeom
from isaacsim.storage.native import get_assets_root_path

timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():                       # replacing a live stage crashed Kit
    timeline.stop()
    raise SystemExit("Simulation was running: stopped it. Run the script again.")

root = get_assets_root_path()                        # NVIDIA's asset store (S3)
robots = {                                           # prim name -> USD to reference
    # The importer names its output after the URDF FILE: go2_description_norotor.urdf
    # (rotor links removed, fixed joints merged) -> go2_py/go2_description_norotor/
    "Mine": "/isaac-sim/work/go2_py/go2_description_norotor/go2_description_norotor.usda",
    "NV_Robots": root + "/Isaac/Robots/Unitree/Go2/go2.usd",
    "NV_IsaacLab": root + "/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd",
    "NV_Multiphysics": root + "/Isaac/Robots_Multiphysics/Unitree/Go2/go2.usda",
}

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

for i, (name, path) in enumerate(robots.items()):
    prim = UsdGeom.Xform.Define(stage, f"/World/{name}").GetPrim()
    UsdGeom.XformCommonAPI(prim).SetTranslate((0.0, 1.0 * i, 0.5))   # side by side, 1 m apart
    prim.GetReferences().AddReference(path)
    vsets = prim.GetVariantSets()
    if "Physics" in vsets.GetNames():               # layered assets: pick the PhysX variant
        vsets.GetVariantSet("Physics").SetVariantSelection("physx")
    stage.Load(prim.GetPath())                      # load payloads now


def drive_value(joint, attr):
    a = joint.GetAttribute(f"drive:angular:physics:{attr}")
    return a.Get() if a and a.HasAuthoredValue() else None


def fmt(v, per_rad=False):
    if v is None:
        return "  -   "
    if per_rad:                                     # per degree -> per radian
        return f"{v:7.3f} ({v * 180 / math.pi:6.2f}/rad)"
    return f"{v:7.2f}"


for name in robots:
    top = stage.GetPrimAtPath(f"/World/{name}")
    prims = list(Usd.PrimRange(top, Usd.TraverseInstanceProxies()))
    revolute = [p for p in prims if p.IsA(UsdPhysics.RevoluteJoint)]
    fixed = [p for p in prims if p.IsA(UsdPhysics.FixedJoint)]
    print(f"\n===== {name}: {len(prims)} prims, {len(revolute)} revolute joints, {len(fixed)} fixed joints")
    if not prims or len(prims) < 3:
        print("  (nothing loaded: check the path/variant)")
        continue

    # One leg's drives and limits
    for leg_joint in ("FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"):
        j = next((p for p in revolute if p.GetName() == leg_joint), None)
        if j is None:
            print(f"  {leg_joint:15s} not found (names: {sorted(p.GetName() for p in revolute)[:4]}...)")
            continue
        rj = UsdPhysics.RevoluteJoint(j)
        print(f"  {leg_joint:15s} stiffness {fmt(drive_value(j, 'stiffness'), True)}  "
              f"damping {fmt(drive_value(j, 'damping'), True)}  "
              f"maxForce {fmt(drive_value(j, 'maxForce'))}  "
              f"limits [{fmt(rj.GetLowerLimitAttr().Get())}, {fmt(rj.GetUpperLimitAttr().Get())}] deg")

    # Masses (rigid bodies with an authored mass)
    total, base = 0.0, None
    for p in prims:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            m = UsdPhysics.MassAPI(p).GetMassAttr().Get() if p.HasAPI(UsdPhysics.MassAPI) else None
            if m:
                total += m
                if p.GetName() == "base":
                    base = m
    print(f"  mass: total {total:.2f} kg (authored on rigid bodies), base {base} kg")

    # Foot collision shape(s)
    feet = [p for p in prims if "FL_foot" in p.GetPath().pathString and p.HasAPI(UsdPhysics.CollisionAPI)]
    for f in feet[:2]:
        extra = ""
        if f.IsA(UsdGeom.Sphere):
            extra = f" radius {UsdGeom.Sphere(f).GetRadiusAttr().Get()}"
        print(f"  FL foot collider: {f.GetTypeName()}{extra}  ({f.GetPath()})")
    if not feet:
        print("  FL foot collider: none found under a path containing 'FL_foot'")

print("\nDone. Compare rows: gains (per rad), maxForce, limits, total mass, foot shape.")
print("Report saved to", REPORT)
sys.stdout = _stdout                                 # restore normal printing
_report.close()
