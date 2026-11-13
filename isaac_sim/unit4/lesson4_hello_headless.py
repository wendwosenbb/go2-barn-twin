# Unit 2, Lesson 4, A2: first standalone (headless) Isaac Sim script.
#
# A cube falls from 1.5 m onto the ground. The script starts the app itself, builds the
# scene, steps the physics itself, MEASURES the cube's height every 10 steps, compares it
# with free fall (z = z0 - g t^2 / 2) and writes a report file. No GUI, no Script Editor.
#
# Run inside the container (next to the streamed app):
#   ssh ubuntu@isaac-sim-ws 'docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/lesson4_hello_headless.py'
#
# ---------------------------------------------------------------------------------------
# MAP OF THE PYTHON APIs YOU MEET IN ISAAC SIM 6.1 (most specific to most general)
#
#   isaacsim.SimulationApp                     starts Kit (the app) from a plain Python
#                                              process. Standalone scripts ONLY. Must come
#                                              first: everything below is loaded by the app.
#
#   isaacsim.core.experimental.*  ("Core API", the 6.x replacement for isaacsim.core.api)
#     .utils.stage    new/open/save stages, add references, get the current stage   [used]
#     .utils.app      play / pause / stop the timeline from code                    [used]
#     .utils.*        more helpers (prims, transforms, backends...)
#     .objects        ready-made scene things: Cube, Sphere, GroundPlane, lights    [alternative shown]
#     .prims          WRAPPERS around existing prims that read/write their state while
#                     simulating, for one prim or thousands at once (paths=...):
#                       XformPrim (poses), GeomPrim (colliders), RigidPrim (bodies:
#                       poses, velocities, mass), Articulation (robots: joint
#                       positions, targets, gains)                                  [RigidPrim used]
#     .materials      visual and physics materials (PreviewSurfaceMaterial...)
#   isaacsim.core.simulation_manager           the physics clock: step size, step(),
#                                              simulation time, physics callbacks     [used]
#   isaacsim.core.rendering_manager            render a frame on demand (cameras, viewports);
#                                              not needed here: nothing looks at the scene
#   isaacsim.storage.native                    where NVIDIA's assets live (get_assets_root_path)
#   isaacsim.asset.importer.urdf               URDF -> USD (Lesson 3)
#   isaacsim.sensors.experimental.rtx/...      cameras, lidars, radars (Lesson 2)
#   isaacsim.ros2.*                            ROS 2 bridge nodes (Lesson 5)
#   omni.*                                     Kit itself: omni.usd (stage context),
#                                              omni.timeline, omni.graph (OmniGraph), omni.kit.*
#   pxr (Usd, UsdGeom, UsdPhysics, UsdLux, Gf) OpenUSD: the scene description itself.
#                                              Works everywhere, also outside Isaac Sim.
#
# Rule of thumb: build the scene with pxr (or .objects), read/drive it while simulating
# with .prims, advance time with simulation_manager.
# ---------------------------------------------------------------------------------------

# 1. Start the app FIRST. Isaac and omni modules can only be imported after this line.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})   # no window, no stream; still uses the GPU

REPORT = "/isaac-sim/work/lesson4_hello_report.txt"   # output goes to a file: there is no GUI
DT = 1.0 / 200.0                                       # physics step: 60 steps per second
STEPS = 400                                           # 2 s of simulated time
Z0, SIZE, G = 1.5, 0.3, 9.81                          # start height, cube edge, gravity

try:
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    # 2. New stage, Z up, metres
    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage(backend="usd")       # the pxr Usd.Stage
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 3. Physics scene, light, ground (pxr, as in Lessons 1-3)
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")      # gravity defaults to -Z, 9.81
    UsdLux.DistantLight.Define(stage, "/World/DistantLight").CreateIntensityAttr(1000.0)
    ground = UsdGeom.Plane.Define(stage, "/World/Ground")       # infinite collision plane
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())             # static: collider, no rigid body
    #   Core API alternative (one line, adds a visual mesh too):
    #   from isaacsim.core.experimental.objects import GroundPlane
    #   GroundPlane("/World/GroundPlane", positions=[0, 0, 0])

    # 4. The cube: shape + rigid body + collider (pxr)
    cube_geom = UsdGeom.Cube.Define(stage, "/World/cube")       # paths start with "/"
    cube_geom.GetSizeAttr().Set(SIZE)
    UsdGeom.XformCommonAPI(cube_geom).SetTranslate(Gf.Vec3d(0.0, 0.0, Z0))
    UsdPhysics.RigidBodyAPI.Apply(cube_geom.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube_geom.GetPrim())
    #   Core API alternative:
    #   from isaacsim.core.experimental.objects import Cube
    #   from isaacsim.core.experimental.prims import GeomPrim
    #   Cube(paths="/World/cube", positions=[[0, 0, Z0]], sizes=SIZE)
    #   GeomPrim(paths="/World/cube", apply_collision_apis=True)

    # 5. A .prims WRAPPER to READ the cube's state while simulating (pxr only gives the
    #    authored start pose; the live pose comes from the physics engine).
    cube = RigidPrim(paths="/World/cube")

    # 6. Step size, then "press Play"
    SimulationManager.set_physics_dt(DT)
    app_utils.play()
    simulation_app.update()                                     # let the app process Play

    # 7. The loop: WE advance time, one physics step at a time, and measure.
    landing_t = ((2 * (Z0 - SIZE / 2)) / G) ** 0.5              # centre reaches SIZE/2 (~0.52 s)
    lines = [f"dt={DT:.5f} s, predicted landing at t={landing_t:.3f} s (step ~{landing_t / DT:.0f})",
             "step   time_s  measured_z  predicted_z  diff_mm"]
    for step in range(1, STEPS + 1):
        SimulationManager.step()                                # one physics step (DT)
        if step % 2 == 0:
            t = SimulationManager.get_simulation_time()         # the sim's own clock, not step*DT
            positions, _ = cube.get_world_poses()               # warp arrays (N, 3), (N, 4)
            z = float(positions.numpy()[0][2])                  # first (only) prim, z component
            z_pred = max(Z0 - 0.5 * G * t * t, SIZE / 2)        # free fall, then resting on the ground
            lines.append(f"{step:4d}  {t:7.3f}  {z:10.4f}  {z_pred:11.4f}  {1000 * (z - z_pred):+8.1f}")

    with open(REPORT, "w") as f:                                # the report: the only output that lasts
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("report written to", REPORT)

finally:
    # 8. Always close the app, even after an error; otherwise the process can hang on the GPU.
    simulation_app.close()
