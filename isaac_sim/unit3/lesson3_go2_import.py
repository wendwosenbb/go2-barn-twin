import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux, PhysxSchema

import omni.timeline

timeline = omni.timeline.get_timeline_interface()

if not timeline.is_stopped():
    timeline.stop()
    raise SystemExit('simulation was running. now stopped. try again')


from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

go2_usd_path = '/isaac-sim/work/go2_py'

go2_usd_scene_path = '/isaac-sim/work/lesson3_go2_scene_py.usda'

go2_urdf_path = '/isaac-sim/work/go2_description/urdf/go2_description.urdf'

# --- Prepare the URDF: drop the motor-rotor links ------------------------------
# Unitree's URDF has 12 "*_rotor" links (0.089 kg each, fixed, no collision) for the motor
# rotors: +1.07 kg, so 16.09 kg instead of the published ~15 kg. NVIDIA's Go2 assets leave
# them out (B6). Their spinning inertia is better modelled later as joint "armature".
# Don't edit Unitree's file: write a copy next to it (package:// mesh paths still resolve).
import xml.etree.ElementTree as ET

tree = ET.parse(go2_urdf_path)
robot = tree.getroot()
rotor_links = {l.get("name") for l in robot.findall("link") if l.get("name").endswith("_rotor")}
for joint in robot.findall("joint"):
    if joint.find("child").get("link") in rotor_links:      # the fixed joint holding a rotor
        robot.remove(joint)
for link in robot.findall("link"):
    if link.get("name") in rotor_links:
        robot.remove(link)
go2_urdf_norotor = go2_urdf_path.replace(".urdf", "_norotor.urdf")
tree.write(go2_urdf_norotor, xml_declaration=True, encoding="utf-8")
print(f"removed {len(rotor_links)} rotor links -> {go2_urdf_norotor}")

go2_asset_path = URDFImporter(URDFImporterConfig(urdf_path= go2_urdf_norotor,
                               usd_path= go2_usd_path,
                               ros_package_paths=[{"name": "go2_description", "path": "/isaac-sim/work/go2_description"}],
                               fix_base= False,
                               merge_fixed_joints= False,      # keep the feet as separate bodies (contact
                                                              # sensing, foot frames). True would be faster,
                                                              # but the 6.1 importer ignores the URDF's
                                                              # dont_collapse and merges the feet too (B7)
                               robot_type= 'Quadruped',
                               collision_from_visuals= False,
                               allow_self_collision= False,
                               joint_drive_type= 'force',
                               joint_target_type= {"(FL|FR|RL|RR)_(hip|thigh|calf)_joint": "position"},
                               override_joint_damping= {"(FL|FR|RL|RR)_(hip|thigh|calf)_joint": 1.15},
                               override_joint_stiffness= {"(FL|FR|RL|RR)_(hip|thigh|calf)_joint": 57.3}
                               )).import_urdf()

ctx = omni.usd.get_context()

ctx.new_stage()

stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetStartTimeCode(0)               # timeline starts at frame 0
stage.SetEndTimeCode(1000000)           # effectively endless: Play won't stop or loop after a few seconds
stage.SetTimeCodesPerSecond(60)         # 60 time codes = 1 s of timeline
stage.SetFramesPerSecond(60)            # playback rate; keep it equal to timeCodesPerSecond

world = UsdGeom.Xform.Define(stage, '/World')

stage.SetDefaultPrim(world.GetPrim())

#scene physics

scene = UsdPhysics.Scene.Define(stage, world.GetPath().AppendPath('PhysicsScene'))

scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0.0,0.0,-1.0))
scene.GetGravityMagnitudeAttr().Set(9.81)

# Physics step: 200 steps/s (5 ms), what Isaac Lab uses for the Go2. At the default 60 Hz,
# PhysX's implicit drive integration adds artificial damping that hides real behaviour (B8).
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
physx_scene.CreateTimeStepsPerSecondAttr(200)

#Ground

ground = UsdGeom.Plane.Define(stage, world.GetPath().AppendPath('Ground'))
ground.GetWidthAttr().Set(100.0)
ground.GetLengthAttr().Set(100.0)
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

#light

light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')
light.GetIntensityAttr().Set(1000)

# Go2 xform prim.

go2 = UsdGeom.Xform.Define(stage, world.GetPath().AppendPath('Go2'))

UsdGeom.XformCommonAPI(go2).SetTranslate(Gf.Vec3d(0.0,0.0,0.35))

go2.GetPrim().GetReferences().AddReference(go2_asset_path)

# The asset keeps its physics in a VARIANT SET "Physics" (mujoco / none / physics / physx),
# each a payload. Choose one explicitly (physx = UsdPhysics joints + PhysX settings),
# then load the payload now, so the joints exist before the loop below looks for them.
go2.GetPrim().GetVariantSet("Physics").SetVariantSelection("physx")
stage.Load(go2.GetPath())
print("Physics variant:", go2.GetPrim().GetVariantSet("Physics").GetVariantSelection())

pose = {"FL_hip_joint": 5.7, "RL_hip_joint": 5.7, "FR_hip_joint": -5.7, "RR_hip_joint": -5.7,
        "FL_thigh_joint": 45.8, "FR_thigh_joint": 45.8, "RL_thigh_joint": 57.3, "RR_thigh_joint": 57.3,
        "FL_calf_joint": -85.9, "FR_calf_joint": -85.9, "RL_calf_joint": -85.9, "RR_calf_joint": -85.9}

joints = {}                                             # name -> prim, found once, reused below
for prim in stage.Traverse():
    if prim.GetPath().pathString.startswith("/World/Go2") and prim.GetName() in pose:
        deg = pose[prim.GetName()]
        prim.GetAttribute("drive:angular:physics:targetPosition").Set(deg)   # where the motor pulls
        prim.GetAttribute("state:angular:physics:position").Set(deg)         # where the joint starts
        prim.GetAttribute("state:angular:physics:velocity").Set(0.0)         # start at rest
        joints[prim.GetName()] = prim

# --- Checks: trust measurements, not assumptions -----------------------------
missing = sorted(set(pose) - set(joints))
if missing:
    raise SystemExit(f"only {len(joints)}/12 leg joints found; missing: {missing}")
print(f"{len(joints)}/12 leg joints set to the standing pose")

# Units check: the importer was given per-RADIAN gains (57.3, 1.15).
# USD angular drives store per-DEGREE values, so a converting importer gives ~1.0 / ~0.02.
for name in ("FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"):
    drive = joints[name]
    print(f"{name}: stiffness {drive.GetAttribute('drive:angular:physics:stiffness').Get()}, "
          f"damping {drive.GetAttribute('drive:angular:physics:damping').Get()}, "
          f"maxForce {drive.GetAttribute('drive:angular:physics:maxForce').Get()}, "
          f"path {drive.GetPath()}")
print("expect stiffness ~1.0 and damping ~0.02 (per degree); ~57.3 / ~1.15 means NOT converted")

# Mass and structure check: compare with NVIDIA's Go2 (B6): 15.02 kg, 4 fixed joints (feet).
robot_prims = list(Usd.PrimRange(go2.GetPrim(), Usd.TraverseInstanceProxies()))
total_mass = sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0.0
                 for p in robot_prims
                 if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.HasAPI(UsdPhysics.MassAPI))
fixed_joints = sorted(p.GetName() for p in robot_prims if p.IsA(UsdPhysics.FixedJoint))
print(f"total mass {total_mass:.2f} kg (NVIDIA 15.02); {len(fixed_joints)} fixed joints: {fixed_joints}")

stage.GetRootLayer().Export(go2_usd_scene_path)
print("imported", go2_asset_path, "-> saved", go2_usd_scene_path)
