from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdLux

import omni.graph.core as og

import omni.usd

import omni.timeline

timeline = omni.timeline.get_timeline_interface()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

if not timeline.is_stopped():
    timeline.stop()
    raise SystemExit("simulation was running. Stopped now.")


stage_path = '/isaac-sim/work/lesson3_carter_py.usda'
ctx = omni.usd.get_context()

carter_usd_path = URDFImporter(URDFImporterConfig(urdf_path= '/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter/urdf/carter.urdf',
                               usd_path= '/isaac-sim/work/carter_py',
                               fix_base= False,
                               robot_type= 'Wheeled',
                               collision_from_visuals= False,
                               allow_self_collision= False,
                               joint_drive_type= 'force',
                               joint_target_type= {"left_wheel|right_wheel": "velocity", "rear_.*": "none"},
                               override_joint_damping= {"left_wheel|right_wheel": 10000.0},     # velocity gain for the drive wheels
                               override_joint_stiffness= {"left_wheel|right_wheel": 0.0},       # explicit: no pull toward an angle
                               )).import_urdf()
print("imported", carter_usd_path)
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

light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')

light.GetIntensityAttr().Set(1000)

scene = UsdPhysics.Scene.Define(stage, world.GetPath().AppendPath('PhysicsScene'))

scene.GetGravityDirectionAttr().Set(Gf.Vec3d(0.0,0.0,-1.0))
scene.GetGravityMagnitudeAttr().Set(9.81)

ground = UsdGeom.Plane.Define(stage, world.GetPath().AppendPath('GroundPlane'))
ground.GetWidthAttr().Set(200)
ground.GetLengthAttr().Set(200)
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

carter = UsdGeom.Xform.Define(stage, world.GetPath().AppendPath('carter'))

UsdGeom.XformCommonAPI(carter).SetTranslate(Gf.Vec3d(0.0,0.0,0.25))

#import the urdf to USD and store it.


carter.GetPrim().GetReferences().AddReference(carter_usd_path)

# Or use the currently imported version stored on vm directly
#carter.GetPrim().GetReferences().AddReference('/isaac-sim/work/carter/carter.usda')



## action graph for differential drive


GRAPH = "/World/DiffDriveGraph"                         # under /World, so it is saved with the robot

keys = og.Controller.Keys                               # names of the edit sections
og.Controller.edit(
    {"graph_path": GRAPH,                               # the OmniGraph prim to create
     "evaluator_name": "execution"},                    # "execution" = Action Graph (event-driven)
    {
        keys.CREATE_NODES: [                            # (short name, node type)
            ("tick", "omni.graph.action.OnPlaybackTick"),                       # fires once per frame while playing
            ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),  # (v, w) -> [left, right] rad/s
            ("artic", "isaacsim.core.nodes.IsaacArticulationController"),      # applies them to the joints
        ],
        keys.CONNECT: [                                 # (source output, destination input)
            ("tick.outputs:tick", "diff.inputs:execIn"),                  # execution wire: run diff every tick
            ("tick.outputs:tick", "artic.inputs:execIn"),                 # execution wire: run artic every tick
            ("diff.outputs:velocityCommand", "artic.inputs:velocityCommand"),  # data wire: wheel speeds
        ],
        keys.SET_VALUES: [                              # ("short.inputs:name", value)
            ("diff.inputs:wheelRadius", 0.24),        
            ("diff.inputs:wheelDistance", 0.628),        
            ("diff.inputs:linearVelocity", 0.5),       
            ("diff.inputs:angularVelocity", 0.0),      
            ("artic.inputs:robotPath", "/World/carter/Geometry/chassis_link"),   # the articulation root
            # In a script the joint list can be set directly: no Constant Token + Make Array
            # (those exist because the GUI has no easy way to type a list). Left first:
            # velocityCommand is [left, right]. Names must match the joints in THIS script.
            ("artic.inputs:jointNames", ["left_wheel", "right_wheel"]),
        ],
    },
)

stage.GetRootLayer().Export(stage_path)

print("imported", carter_usd_path, "-> saved", stage_path)


