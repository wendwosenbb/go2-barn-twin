# Unit 2 proof (Lesson 6), process 1 of 3: the SIMULATOR.
#
# Headless, standalone (Lesson 4): opens the barn, adds Carter with a Go2-like sensor mount
# (camera + 2D lidar), builds the ROS 2 graphs (Lesson 5), advances the APP frame by frame so
# both physics AND the graphs run, watches the robot, and writes a PASS/FAIL report.
# It does NOT steer: the controller (aisle_driver.py, ROS 2 container) does, over /cmd_vel.
#
# Run (run_proof.sh does this):
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/proof_sim.py
# BEFORE running: the streamed GUI app must NOT be playing a ROS scene. Two Kit processes
# publishing /clock, /odom and /tf at once would mix two simulations on the same topics.
#
# Marks:  [VERIFIED]   checked in the 6.1 container in Lessons 3-5 (or in a saved scene)
#         [UNVERIFIED] my best knowledge; check on the VM before relying on it
#
# Pitfalls:
#  1. The connect key is keys.CONNECT (there is no CREATE_CONNECTIONS; Lesson 5 pitfall 4).
#  2. AddReference takes ONE asset path, a string: AddReference(CARTER), not a list.
#  3. The Carter import may not have a "Physics" variant set (Lesson 3's Carter scene never
#     selected one and worked). SetVariantSelection on a set that doesn't exist silently
#     authors a meaningless selection, so check HasVariantSet first.
#  4. Error messages copied from another script mislead: each one names what it checks.
#
# Improvements:
#  a. Spawn at x = 0.5, not 0: the surveyed bollard BOLLARD_02 stands at (-0.42, 0.39), right
#     behind the start spot in this aisle. Equipment has no colliders yet, so Carter would
#     overlap it silently.
#  b. The frame step isn't SET (the 6.1 call is unverified): it's MEASURED from the first
#     frames and reported. That also answers the timeCodesPerSecond experiment.
#  c. The report ends with "RESULT: PASS" or "RESULT: FAIL", and the exit code says the same.
#     run_proof.sh should read the report line: in case simulation_app.close() ends the
#     process with its own exit code (unverified), the report is the reliable signal.

from isaacsim import SimulationApp

# --- 1. Start the app, enable the ROS 2 bridge --------------------------------------------
simulation_app = SimulationApp({"headless": True})                          # [VERIFIED]
# Imports of Isaac Sim modules only work AFTER SimulationApp exists (Lesson 4).
import isaacsim.core.experimental.utils.app as app_utils
# The GUI app enables the ROS 2 bridge at start-up (Lesson 5 log); a standalone app may not.
app_utils.enable_extension("isaacsim.ros2.bridge")                         # [UNVERIFIED]
#   Check in the output: the bridge's start-up line (internal Jazzy libraries loaded), and no
#   "unknown node type isaacsim.ros2.bridge..." when the graphs are built.
simulation_app.update()   # let the extension finish loading before building graphs


# --- 2. Settings in one place -------------------------------------------------------------
BARN = "/isaac-sim/work/barn_usd/barns/farm1_barn1/barn_sim.usda"
CARTER = "/isaac-sim/work/carter_py/carter_1/carter.usda"   # Lesson 3 Python import [VERIFIED]
#          (wheel drives: velocity, damping 10000; rear casters passive)
REPORT = "/isaac-sim/work/proof_report.txt"
SAVED_SCENE = "/isaac-sim/work/proof_scene.usda"      # a NEW file; barn_sim.usda stays untouched
AISLE_Y = 0.31            # centre of the aisle between drinker y=-0.54 and feeder y=+1.16
SPAWN = (0.5, AISLE_Y, 0.25)   # x 0.5: clear of BOLLARD_02 (improvement a); z: Lesson 3 height
GOAL_X = 30.0             # world x to reach; well before the brooding curtain at x=76.7
SIM_SECONDS = 90.0        # time budget: 30 m at 0.5 m/s = 60 s + margin
SETTLE_S = 1.0            # first second: Carter drops 0.25 m -> settles; reference height after
STILL_S = 2.0             # goal reached + standing still this long -> finished early
LOG_EVERY_S = 1.0         # one report row per sim second
FRAME_DT = 1 / 60         # EXPECTED sim time per app frame (measured, see section 9)
# Pass criteria (decided BEFORE running):
MAX_LATERAL = 0.30        # m, |y - AISLE_Y| over the whole run after settling
MAX_TILT_DEG = 15.0       # deg, roll and pitch of the base
MAX_DZ = 0.05             # m, base height change from its settled value (no tipping/climbing)


# --- 3. try / finally: close the app even after an error -----------------------------------
try:
    # --- 4. Imports that need the running app; open the barn ------------------------------
    import math
    import time

    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.graph.core as og
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.sensors.experimental.rtx import Lidar                      # [VERIFIED, L2]
    from isaacsim.storage.native import get_assets_root_path                 # [VERIFIED, L2]
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from usdrt import Sdf as RtSdf          # OmniGraph relationship inputs take usdrt paths

    lines = []                              # the report, built as we go (Lesson 4)

    def log(msg):
        print(msg)
        lines.append(msg)

    ok, stage = stage_utils.open_stage(BARN)                                  # [VERIFIED]
    if not ok or stage is None:
        raise SystemExit(f"could not open {BARN}")
    log(f"opened {BARN}")
    # Exactly one physics scene, and its step (Lesson 4)
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise SystemExit(f"expected 1 PhysicsScene, found {len(scenes)}")
    rate = scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get()
    log(f"physics scene {scenes[0].GetPath()}: {rate} steps/s")

    # --- 5. Add Carter (Lesson 3 pattern) -------------------------------------------------
    carter = UsdGeom.Xform.Define(stage, "/World/Carter")      # wrapper: placement lives here
    UsdGeom.XformCommonAPI(carter).SetTranslate(Gf.Vec3d(*SPAWN))
    carter.GetPrim().GetReferences().AddReference(CARTER)      # one path (str), not a list
    vsets = carter.GetPrim().GetVariantSets()
    if vsets.HasVariantSet("Physics"):                         # select only if the set exists
        vsets.GetVariantSet("Physics").SetVariantSelection("physx")
        log("Carter Physics variant: physx")
    else:
        log("Carter has no Physics variant set (physics is authored directly)")
    stage.Load(carter.GetPath())
    # Find, don't guess: the articulation root (Lesson 3: .../Geometry/chassis_link)
    roots = [p for p in Usd.PrimRange(carter.GetPrim()) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise SystemExit(f"expected 1 articulation root under /World/Carter, found {len(roots)}")
    CHASSIS = str(roots[0].GetPath())                          # robot body = TF frame "chassis_link"
    CHASSIS_FRAME = roots[0].GetName()
    log(f"Carter at {SPAWN}; articulation root {CHASSIS}")
    # The wheel joints must exist under these names (the drive graph uses them)
    joint_names = {p.GetName() for p in Usd.PrimRange(carter.GetPrim()) if p.IsA(UsdPhysics.Joint)}
    for j in ("left_wheel", "right_wheel"):
        if j not in joint_names:
            raise SystemExit(f"joint {j} not found; joints are {sorted(joint_names)}")

    # --- 6. The sensor mount on Carter ----------------------------------------------------
    # chassis_link is an UNSCALED rigid body, so the mount is simply its CHILD: it moves with
    # the chassis; no extra body, no fixed joint (Lesson 2 rule: parent by role).
    MOUNT = f"{CHASSIS}/Front_Sensor"
    CAMERA = f"{MOUNT}/Camera"
    LIDAR = f"{MOUNT}/Lidar"
    mount = UsdGeom.Xform.Define(stage, MOUNT)
    # Front-top of the chassis. [CHECK in the GUI on proof_scene.usda that it clears the body;
    # the numbers are my estimate of Carter's collision box, x -0.35..0.20, top z 0.42]
    UsdGeom.XformCommonAPI(mount).SetTranslate(Gf.Vec3d(0.25, 0.0, 0.45))
    cam = UsdGeom.Camera.Define(stage, CAMERA)
    cam_xf = UsdGeom.Xformable(cam)
    cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.06, 0.0, 0.0))
    cam_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.5, 0.5, -0.5, -0.5))
    cam.CreateFocalLengthAttr(18.147562)                    # same field of view as Lessons 2/5
    cam.CreateHorizontalApertureAttr(20.955)                # explicit (USD default, mm)
    cam.CreateVerticalApertureAttr(15.716)                  # 20.955 x 480/640: square pixels
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000000))   # see from 1 cm
    # Lidar (Lesson 2): an OmniLidar prim referencing NVIDIA's 2D rotary asset, then wrapped
    lidar_prim = stage.DefinePrim(LIDAR, "OmniLidar")
    lidar_prim.GetReferences().AddReference(
        get_assets_root_path() + "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda")
    lidar = Lidar(LIDAR, translations=[[0.0, 0.0, 0.1]])    # keep the object alive (Lesson 2)
    # Note: range_min 1.0 m (Lesson 5): fine for the proof, wrong for the real barn.
    log(f"sensors: {CAMERA} (640x480, fy = fx), {LIDAR}")

    # --- 7. ROS graphs (Lesson 5, adapted to Carter) --------------------------------------
    keys = og.Controller.Keys
    DRIVE = "/World/DriveGraph"
    ROS = "/World/RosGraph"
    SENSORS = "/World/SensorsGraph"

    # Drive: /cmd_vel -> (v, w) -> wheel speeds -> wheel joints. All nodes new: short names.
    og.Controller.edit(
        {"graph_path": DRIVE, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("lin", "omni.graph.nodes.BreakVector3"),
                ("ang", "omni.graph.nodes.BreakVector3"),
                ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("artic", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [                                     # (not CREATE_CONNECTIONS)
                ("tick.outputs:tick", "twist.inputs:execIn"),
                ("tick.outputs:tick", "diff.inputs:execIn"),
                ("tick.outputs:tick", "artic.inputs:execIn"),
                ("twist.outputs:linearVelocity", "lin.inputs:tuple"),
                ("twist.outputs:angularVelocity", "ang.inputs:tuple"),
                ("lin.outputs:x", "diff.inputs:linearVelocity"),       # forward = linear.x
                ("ang.outputs:z", "diff.inputs:angularVelocity"),      # turn = angular.z
                ("diff.outputs:velocityCommand", "artic.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("twist.inputs:topicName", "cmd_vel"),
                ("diff.inputs:wheelRadius", 0.24),              # Lesson 3 values [CHECK from asset]
                ("diff.inputs:wheelDistance", 0.628),
                ("diff.inputs:maxLinearSpeed", 1.0),            # safety limits inside the sim,
                ("diff.inputs:maxAngularSpeed", 1.0),           # whatever the controller sends
                ("artic.inputs:robotPath", CHASSIS),            # the articulation root
                ("artic.inputs:jointNames", ["left_wheel", "right_wheel"]),   # [left, right]
            ],
        },
    )

    # Clock, odometry, TF (Lesson 5 section 4, frames renamed for Carter)
    og.Controller.edit(
        {"graph_path": ROS, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("odomc", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("tfs", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("tick.outputs:tick", "odomc.inputs:execIn"),
                ("tick.outputs:tick", "odom.inputs:execIn"),
                ("tick.outputs:tick", "tf.inputs:execIn"),
                ("tick.outputs:tick", "tfs.inputs:execIn"),
                # ONE time source for every ROS message
                ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
                ("simtime.outputs:simulationTime", "odom.inputs:timeStamp"),
                ("simtime.outputs:simulationTime", "tf.inputs:timeStamp"),
                ("simtime.outputs:simulationTime", "tfs.inputs:timeStamp"),
                ("odomc.outputs:position", "odom.inputs:position"),
                ("odomc.outputs:orientation", "odom.inputs:orientation"),
                ("odomc.outputs:linearVelocity", "odom.inputs:linearVelocity"),
                ("odomc.outputs:angularVelocity", "odom.inputs:angularVelocity"),
            ],
            keys.SET_VALUES: [
                ("odomc.inputs:chassisPrim", [RtSdf.Path(CHASSIS)]),
                ("odom.inputs:odomFrameId", "World"),               # = TF names
                ("odom.inputs:chassisFrameId", CHASSIS_FRAME),      # "chassis_link"
                ("tf.inputs:parentPrim", [RtSdf.Path("/World")]),
                ("tf.inputs:targetPrims", [RtSdf.Path(CHASSIS)]),   # articulation: chassis + wheels
                # Front_Sensor is a plain child now (not a link): static TF chassis -> sensors
                ("tfs.inputs:parentPrim", [RtSdf.Path(CHASSIS)]),
                ("tfs.inputs:targetPrims", [RtSdf.Path(CAMERA), RtSdf.Path(LIDAR)]),
                ("tfs.inputs:staticPublisher", True),
                ("tfs.inputs:topicName", "tf_static"),              # default is "tf": name the topic
            ],
        },
    )

    # Sensors (Lesson 5 section 5). frameSkipCount 1 = publish every 2nd frame: 30 Hz in SIM
    # time, like a real camera, and half the bag size (quiz Q12). Input exists on both camera
    # helpers in the Lesson 5 scenes [VERIFIED].
    og.Controller.edit(
        {"graph_path": SENSORS, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("rp_cam", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("img", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("rp_lidar", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "rp_cam.inputs:execIn"),
                ("tick.outputs:tick", "rp_lidar.inputs:execIn"),
                ("rp_cam.outputs:execOut", "img.inputs:execIn"),
                ("rp_cam.outputs:execOut", "info.inputs:execIn"),
                ("rp_cam.outputs:renderProductPath", "img.inputs:renderProductPath"),
                ("rp_cam.outputs:renderProductPath", "info.inputs:renderProductPath"),
                ("rp_lidar.outputs:execOut", "scan.inputs:execIn"),
                ("rp_lidar.outputs:renderProductPath", "scan.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("rp_cam.inputs:cameraPrim", [RtSdf.Path(CAMERA)]),
                ("rp_cam.inputs:width", 640),
                ("rp_cam.inputs:height", 480),
                ("rp_lidar.inputs:cameraPrim", [RtSdf.Path(LIDAR)]),
                ("img.inputs:type", "rgb"),
                ("img.inputs:topicName", "image_raw"),
                ("img.inputs:frameId", "Camera"),
                ("img.inputs:frameSkipCount", 1),
                ("info.inputs:topicName", "camera_info"),
                ("info.inputs:frameId", "Camera"),
                ("info.inputs:frameSkipCount", 1),             # same rate as the images
                ("scan.inputs:type", "laser_scan"),
                ("scan.inputs:topicName", "scan"),
                ("scan.inputs:frameId", "Lidar"),
            ],
        },
    )
    log("graphs built: DriveGraph, RosGraph, SensorsGraph")

    # --- 8. Wiring check: every key input exactly ONE wire (Lesson 5) ---------------------
    MUST_HAVE_ONE_WIRE = [
        f"{DRIVE}/diff.inputs:linearVelocity",
        f"{DRIVE}/diff.inputs:angularVelocity",
        f"{DRIVE}/artic.inputs:velocityCommand",
        f"{ROS}/clock.inputs:timeStamp",
        f"{ROS}/odom.inputs:timeStamp",
        f"{ROS}/odom.inputs:position",
        f"{ROS}/odom.inputs:orientation",
        f"{ROS}/tf.inputs:timeStamp",
        f"{ROS}/tfs.inputs:timeStamp",
        f"{SENSORS}/img.inputs:renderProductPath",
        f"{SENSORS}/info.inputs:renderProductPath",
        f"{SENSORS}/scan.inputs:renderProductPath",
    ]
    problems = []
    for path in MUST_HAVE_ONE_WIRE:
        names = [s.get_path() for s in og.Controller.attribute(path).get_upstream_connections()]
        if len(names) != 1:
            problems.append(f"{path} <- {names}")
    if problems:
        raise SystemExit(f"wiring check failed (not running): {problems}")
    log(f"wiring check: {len(MUST_HAVE_ONE_WIRE)} inputs ok")

    # --- 9. Play; measure the clocks ------------------------------------------------------
    # Physics dt comes from the barn's PhysicsScene (200 Hz) [VERIFIED, Lesson 4].
    # The frame step (sim time per app frame) is NOT set here: the 6.1 call is unverified
    # (improvement b). It's measured below and compared with FRAME_DT.
    log(f"physics dt {SimulationManager.get_physics_dt():.5f} s")
    app_utils.play()                                                         # [VERIFIED]
    simulation_app.update()                 # process Play (one app frame)

    robot = Articulation(CHASSIS)           # after play: physics handles exist [VERIFIED]

    # Measure: sim time added per app frame, over 10 frames
    t_a = SimulationManager.get_simulation_time()
    for _ in range(10):
        simulation_app.update()
    frame_dt = (SimulationManager.get_simulation_time() - t_a) / 10
    log(f"measured frame step {frame_dt:.5f} s = 1/{1 / frame_dt:.1f} "
        f"(expected {FRAME_DT:.5f}); physics steps per frame {frame_dt / SimulationManager.get_physics_dt():.2f}")

    def read_pose():
        """Base position (x, y, z) and roll, pitch in degrees."""
        pos, quat = robot.get_world_poses()                   # warp arrays [VERIFIED, L4]
        x, y, z = (float(v) for v in pos.numpy()[0])
        w, qx, qy, qz = (float(v) for v in quat.numpy()[0])   # (w, x, y, z)
        roll = math.degrees(math.atan2(2 * (w * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * qy - qz * qx)))))
        return x, y, z, roll, pitch

    # --- 10/11. The loop: advance the APP, not physics alone ------------------------------
    # simulation_app.update() = one app frame: physics catches up (several 5 ms steps),
    # graphs tick, sensors render, ROS publishes. Do NOT also call SimulationManager.step()
    # (Lesson 4's double step).
    t0 = SimulationManager.get_simulation_time()
    wall0 = time.time()
    z_ref = None                    # settled base height
    max_lat = 0.0
    max_tilt = 0.0
    max_dz = 0.0
    x_max = -1e9
    still_since = None              # sim time when the robot stopped after the goal
    last_log = -1e9
    last_xy = None
    log("    t_s       x_m      y_m     z_m   roll  pitch   speed_m/s")
    while True:
        simulation_app.update()
        t = SimulationManager.get_simulation_time() - t0
        x, y, z, roll, pitch = read_pose()
        x_max = max(x_max, x)
        if t >= SETTLE_S:           # criteria count only after Carter has settled
            if z_ref is None:
                z_ref = z
                log(f"settled base height {z_ref:.3f} m")
            max_lat = max(max_lat, abs(y - AISLE_Y))
            max_tilt = max(max_tilt, abs(roll), abs(pitch))
            max_dz = max(max_dz, abs(z - z_ref))
        if t - last_log >= LOG_EVERY_S:
            speed = 0.0 if last_xy is None else math.hypot(x - last_xy[0], y - last_xy[1]) / (t - last_log)
            log(f"{t:7.2f}  {x:8.3f} {y:8.3f} {z:7.3f} {roll:6.1f} {pitch:6.1f}   {speed:6.3f}")
            # Finished early: past the goal and (almost) not moving since the last row
            if x >= GOAL_X and speed < 0.01:
                still_since = t if still_since is None else still_since
            else:
                still_since = None
            last_log, last_xy = t, (x, y)
        if still_since is not None and t - still_since >= STILL_S:
            log(f"robot stopped past the goal: ending at t = {t:.2f} s")
            break
        if t >= SIM_SECONDS:
            log(f"time budget {SIM_SECONDS} s used up")
            break
    wall = time.time() - wall0

    # --- 12. Report: decide PASS / FAIL ---------------------------------------------------
    checks = [
        ("reached goal x", f"{x_max:.2f} m", f">= {GOAL_X}", x_max >= GOAL_X),
        ("max |y - aisle|", f"{max_lat:.3f} m", f"<= {MAX_LATERAL}", max_lat <= MAX_LATERAL),
        ("max roll/pitch", f"{max_tilt:.1f} deg", f"<= {MAX_TILT_DEG}", max_tilt <= MAX_TILT_DEG),
        ("max height change", f"{max_dz:.3f} m", f"<= {MAX_DZ}", max_dz <= MAX_DZ),
        ("finished on its own", "yes" if still_since is not None else "no", "yes",
         still_since is not None),
    ]
    log("")
    log(f"{'criterion':22s} {'measured':>12s} {'limit':>10s}  result")
    for name, measured, limit, good in checks:
        log(f"{name:22s} {measured:>12s} {limit:>10s}  {'ok' if good else 'FAIL'}")
    passed = all(c[3] for c in checks)
    log(f"sim time {t:.1f} s, wall time {wall:.1f} s, RTF {t / wall:.2f}")
    log("Not checked here (run_proof.sh does): the bag contains all topics.")
    log(f"RESULT: {'PASS' if passed else 'FAIL'}")     # run_proof.sh reads this line
    with open(REPORT, "w") as f:                     # the result: write it first (Lesson 4)
        f.write("\n".join(lines) + "\n")
    print("report written to", REPORT)

    app_utils.stop()                                 # save the authored start state, not mid-run
    simulation_app.update()
    saved = stage_utils.save_stage(SAVED_SCENE)      # [VERIFIED, L4] new file, paths fixed
    print("scene saved:", saved, SAVED_SCENE)
    exit_code = 0 if passed else 1

# --- 13. finally: close the app, then exit with the result --------------------------------
finally:
    simulation_app.close()

raise SystemExit(exit_code)    # only reached without an exception (errors exit non-zero anyway)
