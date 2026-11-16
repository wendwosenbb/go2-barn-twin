# Unit 2, Lesson 1, Part B: the GUI steps "Stage setup" + "Assemble a simple
# robot" (Isaac Sim 6.1 tutorials 1-2), done in Python instead of clicks.
#
# Run it INSIDE Isaac Sim: Window > Script Editor, paste (or File > Open this
# file from /isaac-sim/work/), then Run (Ctrl+Enter).
#
# Only pxr (OpenUSD) APIs are used, so this is the same USD authoring as Unit 1.
# The GUI physics menus write exactly these schemas; compare with the
# Raw USD Properties of your GUI-built robot.
import omni.usd                                         # Kit's handle on the open stage
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics  # USD core, lights, physics

OUT = "/isaac-sim/work/lesson1_simple_robot_py.usda"    # text USD on /data, to diff with the GUI file

# --- Stage setup (GUI: File > New, Edit > Preferences > Stage) ----------------
import omni.timeline                                    # Play / Pause / Stop state
timeline = omni.timeline.get_timeline_interface()       # the app's timeline
if not timeline.is_stopped():                           # simulation running or paused?
    timeline.stop()                                     # replacing a live stage crashed Kit
    raise SystemExit("Simulation was running: stopped it. Run the script again.")

ctx = omni.usd.get_context()                            # the app's USD context
ctx.new_stage()                                         # File > New
stage = ctx.get_stage()                                 # the empty stage it opened

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)         # Z up, as in the barn
UsdGeom.SetStageMetersPerUnit(stage, 1.0)               # 1 unit = 1 m (no 100x bugs)
world = UsdGeom.Xform.Define(stage, "/World")           # root Xform
stage.SetDefaultPrim(world.GetPrim())                   # defaultPrim, as in barn_usd

light = UsdLux.DistantLight.Define(stage, "/World/Light")  # a script stage has no light
light.CreateIntensityAttr(1000.0)                       # bright enough for RTX

# --- Physics scene (GUI: Create > Physics > Physics Scene) --------------------
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")  # one per stage
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))     # down is -Z
scene.CreateGravityMagnitudeAttr(9.81)                  # m/s^2, because meters
physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())  # PhysX-specific settings
physx.CreateEnableGPUDynamicsAttr(False)                # tutorial: CPU is enough here
physx.CreateBroadphaseTypeAttr("MBP")                   # tutorial: MBP broadphase

# --- Ground (GUI: Create > Physics > Ground Plane) ----------------------------
# Same layout the GUI writes: a visual mesh plus a separate collision-only plane.
# A thin box collider instead made the wheels rock on landing (Lesson 1 test):
# an infinite analytic plane gives the solver simpler, steadier contacts.
UsdGeom.Xform.Define(stage, "/World/GroundPlane")       # groups visual + collider
vis = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/CollisionMesh")  # what you see
vis.CreatePointsAttr([(-25, -25, 0), (25, -25, 0), (25, 25, 0), (-25, 25, 0)])  # 50 x 50 m
vis.CreateFaceVertexCountsAttr([4])                     # one quad...
vis.CreateFaceVertexIndicesAttr([0, 1, 2, 3])           # ...through all 4 points
vis.CreateDisplayColorAttr([(0.5, 0.5, 0.5)])           # grey
plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane/CollisionPlane")  # infinite plane
plane.CreateAxisAttr(UsdGeom.Tokens.z)                  # its normal points up (+Z)
plane.CreatePurposeAttr(UsdGeom.Tokens.guide)           # not rendered; physics only
UsdPhysics.CollisionAPI.Apply(plane.GetPrim())          # static collider: no RigidBodyAPI

# --- Robot parts (GUI: Create > Xform, Create > Shape > Cube / Cylinder) ------
def xform(path, translate, rotate=(0.0, 0.0, 0.0)):
    """An Xform at `translate` (m), rotated by XYZ Euler angles (degrees)."""
    x = UsdGeom.Xform.Define(stage, path)               # grouping prim
    api = UsdGeom.XformCommonAPI(x)                     # common TRS ops
    api.SetTranslate(Gf.Vec3d(*translate))              # position of the part
    api.SetRotate(Gf.Vec3f(*rotate))                    # orientation of the part
    return x

def rigid(prim):
    """GUI: + Add > Physics > Rigid Body with Colliders Preset."""
    UsdPhysics.RigidBodyAPI.Apply(prim)                 # it moves under physics
    UsdPhysics.CollisionAPI.Apply(prim)                 # and it collides

xform("/World/body", (0.0, 0.0, 1.0))                   # chassis frame, 1 m up
cube = UsdGeom.Cube.Define(stage, "/World/body/body")   # chassis shape
cube.CreateSizeAttr(1.0)                                # 1 m cube before scaling
UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(2.0, 1.0, 0.5))  # 2 x 1 x 0.5 m
rigid(cube.GetPrim())

for name, y in (("wheel_left", 1.5), ("wheel_right", -1.5)):
    xform(f"/World/{name}", (0.0, y, 1.0), (90.0, 0.0, 0.0))  # axle along Y
    cyl = UsdGeom.Cylinder.Define(stage, f"/World/{name}/{name}")  # wheel shape
    cyl.CreateRadiusAttr(0.5)                           # 1 m diameter
    cyl.CreateHeightAttr(1.0)                           # 1 m wide
    rigid(cyl.GetPrim())

# --- Save (GUI: File > Save As) -----------------------------------------------
stage.GetRootLayer().Export(OUT)                        # write the stage to /data
print(f"built the simple robot; saved {OUT}. Press Play: the 3 parts fall apart.")
