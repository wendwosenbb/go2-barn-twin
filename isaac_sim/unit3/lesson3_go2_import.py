import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux

import omni.timeline

timeline = omni.timeline.get_timeline_interface()

if not timeline.is_stopped():
    timeline.stop()
    raise SystemExit('simulation was running. now stopped. try again')


from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

go2_usd_path = '/isaac-sim/work/go2_py'

go2_usd_scene_path = '/isaac-sim/work/lesson3_go2_scene_py.usda'

go2_urdf_path = '/isaac-sim/work/go2_description/urdf/go2_description.urdf'

go2_asset_path = URDFImporter(URDFImporterConfig(urdf_path= go2_urdf_path,
                               usd_path= go2_usd_path,
                               ros_package_paths=[{"name": "go2_description", "path": "/isaac-sim/work/go2_description"}],
                               fix_base= False,
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

stage.GetRootLayer().Export(go2_usd_scene_path)
print("imported", go2_asset_path, "-> saved", go2_usd_scene_path)
