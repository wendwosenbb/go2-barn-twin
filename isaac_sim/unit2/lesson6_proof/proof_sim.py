# Unit 2 proof (Lesson 6), process 1 of 3: the SIMULATOR.
#
# Headless, standalone (Lesson 4): opens the barn, adds Carter with a Go2-like sensor mount
# (camera + 2D lidar), builds the ROS 2 graphs (Lesson 5), advances the APP frame by frame so
# both physics AND the graphs run, watches the robot, and writes a PASS/FAIL report.
# It does NOT steer: the controller (aisle_driver.py, ROS 2 container) does, over /cmd_vel.
#
# Run (run_proof.sh does this):
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/proof_sim.py
#
# Marks:  [VERIFIED]   checked in the 6.1 container in Lessons 3-5
#         [UNVERIFIED] my best knowledge; check on the VM before relying on it


# --- 1. Start the app, enable the ROS 2 bridge --------------------------------------------
# SimulationApp({"headless": True})                                          [VERIFIED]
# The GUI app enables the ROS 2 bridge at start-up (Lesson 5 log); a standalone app may not.
# import isaacsim.core.experimental.utils.app as app_utils
# app_utils.enable_extension("isaacsim.ros2.bridge")                         [UNVERIFIED]
#   (SimulationManager's own source calls app_utils.enable_extension(...) for another
#    extension, so the function exists; whether the bridge needs it headless: check)
# simulation_app.update()  -> let the extension finish loading before building graphs


# --- 2. Settings in one place -------------------------------------------------------------
# BARN   = /isaac-sim/work/barn_usd/barns/farm1_barn1/barn_sim.usda
# CARTER = /isaac-sim/work/carter_py/carter_1/carter.usda   (Lesson 3 Python import:
#          wheel drives with damping 10000, caster passive)  -> check the folder name on the VM
# SPAWN  = (0.0, AISLE_Y, 0.25)   barn origin (start spot), Carter's computed spawn height
# AISLE_Y = 0.31                  centre of the aisle between drinker y=-0.54 and feeder y=+1.16
# GOAL_X  = 30.0                  well before the brooding curtain at x=76.7
# SIM_SECONDS = 90.0              time budget: 30 m at 0.5 m/s = 60 s + margin
# FRAME_DT = 1/60                 sim time per app frame = graph/sensor/control rate (Lesson 6 notes)
# Pass criteria (decided BEFORE running): x >= GOAL_X; |y - AISLE_Y| <= 0.3 all the way;
#   not tipped (base roll/pitch < 15 deg); base height stays within a few cm of its start.
# REPORT, SAVED_SCENE under /isaac-sim/work/


# --- 3. try / finally: close the app even after an error -----------------------------------


# --- 4. Open the barn, check its physics scene ----------------------------------------------
# ok, stage = stage_utils.open_stage(BARN)                                   [VERIFIED]
# exactly one UsdPhysics.Scene, read physxScene:timeStepsPerSecond (200)      [VERIFIED]


# --- 5. Add Carter (Lesson 3 pattern) -------------------------------------------------------
# Wrapper Xform /World/Carter at SPAWN, AddReference(CARTER),
# select the "Physics" variant "physx", stage.Load(path)                     [VERIFIED pattern]
# Find, don't guess: the articulation root (prim with ArticulationRootAPI: chassis_link) and
# the wheel joints left_wheel / right_wheel (Lesson 3).


# --- 6. The sensor mount on Carter ----------------------------------------------------------
# Carter's chassis_link is an UNSCALED Xform rigid body (unlike Lesson 2's scaled Cube), so the
# mount can simply be a CHILD of it: children of a rigid body move with it, no extra body and
# no fixed joint needed (the Lesson 2 rule: parent by role).
#   Front_Sensor  Xform under chassis_link, at the front-top of the chassis
#                 (chassis collision box: x -0.35..0.20, z 0.0..0.42 in chassis_link's frame;
#                  e.g. translate (0.25, 0, 0.45) -> check it clears the box in the GUI)
#   Camera        under Front_Sensor: translate (0.06, 0, 0), orient quaternion
#                 (0.5, 0.5, -0.5, -0.5), focalLength 18.147562, clippingRange (0.01, 1e7),
#                 verticalAperture 15.716 (square pixels at 640x480: Lesson 5 warning)
#   Lidar         under Front_Sensor: OmniLidar referencing NVIDIA's Example_Rotary_2D asset,
#                 translate (0, 0, 0.1), wrapped with Lidar(...)            [VERIFIED, Lesson 2]
#                 Note: range_min 1.0 m (Lesson 5): fine for the proof, wrong for the real barn.


# --- 7. ROS graphs (Lesson 5, adapted to Carter) --------------------------------------------
# One new graph each, og.Controller.edit with a dict {"graph_path", "evaluator_name"}:
#   DriveGraph:  tick -> ROS2SubscribeTwist(cmd_vel) -> 2 x BreakVector3 -> DifferentialController
#                (wheelRadius 0.24, wheelDistance 0.628, maxLinearSpeed 1.0, maxAngularSpeed 1.0)
#                -> IsaacArticulationController (robotPath = chassis_link path,
#                   jointNames ["left_wheel", "right_wheel"])
#   RosGraph:    clock; IsaacComputeOdometry (chassisPrim) -> ROS2PublishOdometry
#                (odomFrameId "World", chassisFrameId "chassis_link" = TF names);
#                TF: parentPrim /World, targetPrims [chassis_link];
#                static TF: parentPrim chassis_link... see note
#   SensorsGraph: render product camera 640x480 -> ROS2CameraHelper (rgb, image_raw, Camera)
#                 + ROS2CameraInfoHelper (camera_info, Camera); camera helper frameSkipCount 1
#                 (every 2nd frame = 30 Hz in sim time, like a real camera, half the bag size);
#                 render product lidar -> ROS2RtxLidarHelper (laser_scan, scan, Lidar)
# Note on static TF: Front_Sensor is now a plain child of chassis_link (not a link), so the
#   static TF publishes chassis_link -> Camera and chassis_link -> Lidar (parentPrim
#   chassis_link, targets Camera, Lidar, topicName "tf_static"!). Check frames in /tf_static.
# Same rules as Lesson 5: one wire per input, full paths for existing nodes, usdrt paths for
#   relationships, ONE simulation-time source for every timeStamp.


# --- 8. Wiring check ---------------------------------------------------------------------
# Lesson 5's MUST_HAVE_ONE_WIRE loop; stop before running if any input fails.


# --- 9. Clock settings, then Play -----------------------------------------------------------
# Physics dt comes from the barn's PhysicsScene (200 Hz).
# Frame step (rendering dt) = FRAME_DT                                       [UNVERIFIED]
#   look for SimulationManager / RenderingManager setters for the rendering dt in 6.1
#   (older API: World(physics_dt=..., rendering_dt=...)); then CONFIRM it by the step
#   between consecutive /clock messages.
# app_utils.play()                                                            [VERIFIED]


# --- 10. Read the robot while running ---------------------------------------------------------
# robot = Articulation(chassis_link path)    after play                     [VERIFIED pattern]
# robot.get_world_poses() -> position (x, y, z) and orientation (w, x, y, z) of the base


# --- 11. The loop: advance the APP, not physics alone --------------------------------------------
# while sim time < SIM_SECONDS:
#     simulation_app.update()          one app frame: physics catches up (several 5 ms steps),
#                                      graphs tick, sensors render, ROS publishes  [Lesson 4/5]
#     (do NOT also call SimulationManager.step(): Lesson 4's double-step)
#     every N frames: read pose; track max |y - AISLE_Y|, max roll/pitch, min/max base z,
#                     furthest x; log a row
#     stop early once x >= GOAL_X and the robot has stood still for ~2 s (controller finished)
# Use SimulationManager.get_simulation_time() for t (sim time, not wall time).


# --- 12. Report: decide PASS / FAIL ----------------------------------------------------------
# One line per criterion: measured value, limit, ok/FAIL; overall PASS only if all ok.
# Also: sim time used, wall time used (time.time() around the loop) -> RTF of this run.
# Write the report first, then app_utils.stop(), save the scene to a NEW file
# (stage_utils.save_stage(SAVED_SCENE)), and exit with code 0 on PASS, 1 on FAIL
# (run_proof.sh reads it).


# --- 13. finally: simulation_app.close() ------------------------------------------------------
