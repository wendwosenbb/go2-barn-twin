from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux, UsdShade
import omni.graph.core as og

# Only pxr (OpenUSD) APIs are used, so this is the same USD authoring as Unit 1.
# The GUI physics menus write exactly these schemas; compare with the
# Raw USD Properties of your GUI-built robot.
import omni.usd 


#define method that appy rigidbodyapi and collision api to a prim

def rigid(prim):
    UsdPhysics.RigidBodyAPI.Apply(prim) # allows gravity from the scene to apply to this prim
    UsdPhysics.CollisionAPI.Apply(prim) # allows collision physics to apply

stage_path = "/isaac-sim/work/lesson2_robot_with_graph_py.usda"

# --- Stage setup (GUI: File > New, Edit > Preferences > Stage) ----------------
import omni.timeline                                    # Play / Pause / Stop state
timeline = omni.timeline.get_timeline_interface()       # the app's timeline
if not timeline.is_stopped():                           # simulation running or paused?
    timeline.stop()                                     # replacing a live stage crashed Kit
    raise SystemExit("Simulation was running: stopped it. Run the script again.")

ctx = omni.usd.get_context()                            # the app's USD context
ctx.new_stage()                                         # File > New
stage = ctx.get_stage()                                 # the empty stage it opened

#create and setup a stage

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

stage.SetStartTimeCode(0)               # timeline starts at frame 0
stage.SetEndTimeCode(1000000)           # effectively endless: Play won't stop or loop after a few seconds
stage.SetTimeCodesPerSecond(60)         # 60 time codes = 1 s of timeline
stage.SetFramesPerSecond(60)            # playback rate; keep it equal to timeCodesPerSecond

#create the world xform prim

UsdGeom.Xform.Define(stage, '/World')

#light
light = UsdLux.DistantLight.Define(stage, "/World/Light")  # a script stage has no light
light.CreateIntensityAttr(1000.0)                       # bright enough for RTX

#adding physics scene for gravity, 

scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')

scene.GetGravityDirectionAttr().Set(Gf.Vec3d(0.0, 0.0, -1.0))
scene.GetGravityMagnitudeAttr().Set(9.81)
# add ground as a plane and apply the physics scene. 

ground_geom = UsdGeom.Plane.Define(stage, '/World/Ground')

ground_geom.GetWidthAttr().Set(30)
ground_geom.GetLengthAttr().Set(30)

ground_geom.CreateAxisAttr(UsdGeom.Tokens.z)

UsdPhysics.CollisionAPI.Apply(ground_geom.GetPrim())

#create simplerobot xform to hold togehter robot parts under one xformable tree

simplerobot = UsdGeom.Xform.Define(stage, '/World/SimpleRobot')
UsdGeom.XformCommonAPI(simplerobot).SetTranslate(Gf.Vec3d(0,0,0.375))
UsdPhysics.ArticulationRootAPI.Apply(simplerobot.GetPrim())

#chasis
cube = UsdGeom.Cube.Define(stage,'/World/SimpleRobot/Cube')
cube.GetSizeAttr().Set(1.0)
UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(2.0, 1.0, 0.5))
rigid(cube.GetPrim())
#create wheels front: left and right, back: left and right

front_left_wheel_prim = UsdGeom.Cylinder.Define(stage, simplerobot.GetPath().AppendPath('front_left_wheel'))
front_left_wheel_prim.GetRadiusAttr().Set(0.5)
front_left_wheel_prim.GetHeightAttr().Set(1.0)
UsdGeom.XformCommonAPI(front_left_wheel_prim).SetScale(Gf.Vec3f(0.75, 0.75, 0.25))
UsdGeom.XformCommonAPI(front_left_wheel_prim).SetTranslate(Gf.Vec3d(0.5, 0.75, 0.0))
UsdGeom.XformCommonAPI(front_left_wheel_prim).SetRotate(Gf.Vec3f(90.0, 0.0, 0.0))
rigid(front_left_wheel_prim.GetPrim())

front_right_wheel_prim = UsdGeom.Cylinder.Define(stage, simplerobot.GetPath().AppendPath('front_right_wheel'))
front_right_wheel_prim.GetRadiusAttr().Set(0.5)
front_right_wheel_prim.GetHeightAttr().Set(1.0)
UsdGeom.XformCommonAPI(front_right_wheel_prim).SetScale(Gf.Vec3f(0.75, 0.75, 0.25))
UsdGeom.XformCommonAPI(front_right_wheel_prim).SetTranslate(Gf.Vec3d(0.5, -0.75, 0.0))
UsdGeom.XformCommonAPI(front_right_wheel_prim).SetRotate(Gf.Vec3f(90.0, 0, 0))
rigid(front_right_wheel_prim.GetPrim())

back_left_wheel_prim = UsdGeom.Cylinder.Define(stage, simplerobot.GetPath().AppendPath('back_left_wheel'))
back_left_wheel_prim.GetRadiusAttr().Set(0.5)
back_left_wheel_prim.GetHeightAttr().Set(1.0)
UsdGeom.XformCommonAPI(back_left_wheel_prim).SetScale(Gf.Vec3f(0.75, 0.75, 0.25))
UsdGeom.XformCommonAPI(back_left_wheel_prim).SetTranslate(Gf.Vec3d(-0.5, 0.75, 0.0))
UsdGeom.XformCommonAPI(back_left_wheel_prim).SetRotate(Gf.Vec3f(90.0, 0,0))
rigid(back_left_wheel_prim.GetPrim())

back_right_wheel_prim = UsdGeom.Cylinder.Define(stage, simplerobot.GetPath().AppendPath('back_right_wheel'))
back_right_wheel_prim.GetRadiusAttr().Set(0.5)
back_right_wheel_prim.GetHeightAttr().Set(1.0)
UsdGeom.XformCommonAPI(back_right_wheel_prim).SetScale(Gf.Vec3f(0.75, 0.75, 0.25))
UsdGeom.XformCommonAPI(back_right_wheel_prim).SetTranslate(Gf.Vec3d(-0.5, -0.75, 0.0))
UsdGeom.XformCommonAPI(back_right_wheel_prim).SetRotate(Gf.Vec3f(90.0, 0,0))
rigid(back_right_wheel_prim.GetPrim())

mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/slippery")   # a Material prim
phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())                          # make it a PHYSICS material
phys.CreateStaticFrictionAttr(0.1)
phys.CreateDynamicFrictionAttr(0.1)
phys.CreateRestitutionAttr(0.0)

for wheel in (front_left_wheel_prim, front_right_wheel_prim):
    UsdShade.MaterialBindingAPI.Apply(wheel.GetPrim()).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")              # purpose "physics": friction, not looks

#create physics: revolute joint
front_left_joint = UsdPhysics.RevoluteJoint.Define(stage, simplerobot.GetPath().AppendPath('FrontLeftJoint'))
front_left_joint.GetBody0Rel().SetTargets([cube.GetPath()])
front_left_joint.GetBody1Rel().SetTargets([front_left_wheel_prim.GetPath()])
front_left_joint.GetAxisAttr().Set('Y')

'''
Setting localpos0 and localpos1

localPos0 and localPos1 are that same point written twice: 
once in body0's coordinates and once in body1's.

┌───────────┬───────────────────────────────────────────────────────────────────┐
│ Attribute │                       Frame it's written in                       │
├───────────┼───────────────────────────────────────────────────────────────────┤
│ localPos0 │ body0's local frame (the Cube), including the Cube's scale        │
├───────────┼───────────────────────────────────────────────────────────────────┤
│ localPos1 │ body1's local frame (the wheel), including its rotation and scale │
└───────────┴───────────────────────────────────────────────────────────────────┘

When the simulation starts, both values convert to the same point in the world. 
The joint then keeps those two points together.

How to work them out for Front_Left:

1. Pivot in world: the wheel centre is SimpleRobot (0, 0, 0.375) + wheel translate (0.5, 0.75, 0), which gives (0.5, 0.75, 0.375).
2. localPos1 (wheel frame): the wheel's own origin is its centre, so (0, 0, 0). Every wheel gets this, because the pivot is at each wheel's origin.
3. localPos0 (Cube frame): take the pivot relative to the Cube's origin, then undo the Cube's scale.
   - The Cube's world origin is (0, 0, 0.375), so the offset is (0.5, 0.75, 0).
   - Divide by the scale (2, 1, 0.5) to get (0.25, 0.75, 0).
   - This is why the value looks "wrong" at first: 0.25 in Cube units is 0.5 m in the world, because the Cube is stretched 2× in X.
   - The Y value of 0.75 lies outside the Cube's ±0.5 extent. That's fine: the point doesn't have to be inside the body.

   General rule: localPos0 = inverse(body0 world transform) × pivot.

   From the file you pasted:

point3f[] points = [(-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), ... ]   ← corners
double3 xformOp:scale = (2, 1, 0.5)

The Cube on screen is 2 m long, but its corners are still stored as ±0.5. Setting the scale never changed those numbers. The renderer and physics multiply them by the scale when they use them:

┌─────────────────────────────────┬─────────────────────┬───────────────────────────────────────────────┐
│ Corner in the file (Cube frame) │ × scale (2, 1, 0.5) │ Where it really is (metres, before translate) │
├─────────────────────────────────┼─────────────────────┼───────────────────────────────────────────────┤
│ (0.5, 0.5, 0.5)                 │ →                   │ (1.0, 0.5, 0.25)                              │
└─────────────────────────────────┴─────────────────────┴───────────────────────────────────────────────┘

So "scale applied to the frame" means: the numbers stay the same, and each axis unit gets longer or shorter. Along the Cube's X axis, one unit is now 2 m, so the stored corner value 0.5 sits 1 m out.

A ruler picture: draw a mark at "0.25" on a rubber ruler, then stretch the ruler to twice its length. The mark still says 0.25, but it's now 0.5 m from zero. The scale is the stretch, and the Cube's frame is the rubber ruler.

Now the joint: localPos0 is stored in exactly the same way as those corners. It's just one more point on the Cube's rubber ruler, so physics multiplies it by the scale too.

┌───────────────────┬──────────────────────────┬─────────────────────┬──────────────────────────────────┐
│                   │ Stored in the Cube frame │ × scale (2, 1, 0.5) │ Real offset from the Cube centre │
├───────────────────┼──────────────────────────┼─────────────────────┼──────────────────────────────────┤
│ Front corner      │ (0.5, 0.5, 0.5)          │ →                   │ (1.0, 0.5, 0.25) m               │
├───────────────────┼──────────────────────────┼─────────────────────┼──────────────────────────────────┤
│ Hinge (localPos0) │ (0.25, 0.75, 0)          │ →                   │ (0.5, 0.75, 0) m ✓ wheel centre  │
└───────────────────┴──────────────────────────┴─────────────────────┴──────────────────────────────────┘

You want the hinge 0.5 m forward. Physics will double whatever you store in X, so you store 0.25. That's all "undoing the scale" means: dividing by 2 now, because physics will multiply by 2 later.

If you stored 0.5, physics would double it to 1.0 m, and the hinge would sit at the front face of the cube instead of at the wheel.
'''
front_left_joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.25, 0.75, 0.0))
front_left_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

front_left_joint.GetLocalRot0Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 0.0).GetQuat()))
front_left_joint.GetLocalRot1Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), -90.0).GetQuat()))
front_left_joint.GetLowerLimitAttr().Set(float('-inf'))
front_left_joint.GetUpperLimitAttr().Set(float('inf'))


# front_right joint
front_right_joint = UsdPhysics.RevoluteJoint.Define(stage, simplerobot.GetPath().AppendPath('FrontRightJoint'))

front_right_joint.GetBody0Rel().SetTargets([cube.GetPath()])
front_right_joint.GetBody1Rel().SetTargets([front_right_wheel_prim.GetPath()])
front_right_joint.GetAxisAttr().Set('Y')

front_right_joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.25, -0.75, 0.0))
front_right_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0,0.0))

front_right_joint.GetLocalRot0Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 0.0).GetQuat()))
front_right_joint.GetLocalRot1Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), -90.0).GetQuat()))
front_right_joint.GetLowerLimitAttr().Set(float('-inf'))
front_right_joint.GetUpperLimitAttr().Set(float('inf'))


#back_left_joint
back_left_joint = UsdPhysics.RevoluteJoint.Define(stage, simplerobot.GetPath().AppendPath('BackLeftJoint'))
back_left_joint.GetBody0Rel().SetTargets([cube.GetPath()])
back_left_joint.GetBody1Rel().SetTargets([back_left_wheel_prim.GetPath()])
back_left_joint.GetAxisAttr().Set('Y')

back_left_joint.GetLocalPos0Attr().Set(Gf.Vec3f(-0.25, 0.75,0.0))
back_left_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

back_left_joint.GetLocalRot0Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 0.0).GetQuat()))
back_left_joint.GetLocalRot1Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), -90.0).GetQuat()))
back_left_joint.GetLowerLimitAttr().Set(float('-inf'))
back_left_joint.GetUpperLimitAttr().Set(float('inf'))

#back_right_joint
back_right_joint = UsdPhysics.RevoluteJoint.Define(stage, simplerobot.GetPath().AppendPath('BackRightJoint'))
back_right_joint.GetBody0Rel().SetTargets([cube.GetPath()])
back_right_joint.GetBody1Rel().SetTargets([back_right_wheel_prim.GetPath()])
back_right_joint.GetAxisAttr().Set('Y')

back_right_joint.GetLocalPos0Attr().Set(Gf.Vec3d(-0.25, -0.75,0.0))
back_right_joint.GetLocalPos1Attr().Set(Gf.Vec3d(0,0,0))

back_right_joint.GetLocalRot0Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 0.0).GetQuat()))
back_right_joint.GetLocalRot1Attr().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), -90.0).GetQuat()))

back_right_joint.GetLowerLimitAttr().Set(float('-inf'))
back_right_joint.GetUpperLimitAttr().Set(float('inf'))


#adding drive to the rear joints

#back_left
back_left_drive = UsdPhysics.DriveAPI.Apply(back_left_joint.GetPrim(), 'angular')
back_left_drive.GetTypeAttr().Set('force')
back_left_drive.GetDampingAttr().Set(10000.0)
back_left_drive.GetStiffnessAttr().Set(0.0)
back_left_drive.GetTargetVelocityAttr().Set(50.0)

#back_right
back_right_drive = UsdPhysics.DriveAPI.Apply(back_right_joint.GetPrim(), 'angular')
back_right_drive.GetTypeAttr().Set('force')
back_right_drive.GetDampingAttr().Set(10000.0)
back_right_drive.GetStiffnessAttr().Set(0.0)
back_right_drive.GetTargetVelocityAttr().Set(50.0)

stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))

# omni graph differential controller.
# Same graph as the hand-built GUI one (Lesson 2, Part A), built in one call.
# One og.Controller.edit = one transaction: create nodes, connect, set values.
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
            ("diff.inputs:wheelRadius", 0.375),         # m: 0.5 cylinder radius x 0.75 scale
            ("diff.inputs:wheelDistance", 1.5),         # m: wheel centres at y = +-0.75
            ("diff.inputs:linearVelocity", 1.0),        # EDIT: forward speed v, m/s
            ("diff.inputs:angularVelocity", 0.0),       # EDIT: turn rate w, rad/s (+ = left turn)
            ("artic.inputs:robotPath", "/World/SimpleRobot"),   # the articulation root
            # In a script the joint list can be set directly: no Constant Token + Make Array
            # (those exist because the GUI has no easy way to type a list). Left first:
            # velocityCommand is [left, right]. Names must match the joints in THIS script.
            ("artic.inputs:jointNames", ["BackLeftJoint", "BackRightJoint"]),
        ],
    },
)
# If the robot doesn't move and the log says the controller has no target,
# set the relationship instead of robotPath (needs `import usdrt` at the top):
#     og.Controller.edit(GRAPH, {keys.SET_VALUES: [
#         ("artic.inputs:targetPrim", [usdrt.Sdf.Path("/World/SimpleRobot")])]})

stage.GetRootLayer().Export(stage_path)
