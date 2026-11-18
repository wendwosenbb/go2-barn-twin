# Unit 2, Lesson 5: the ROS 2 graphs of Parts B and C, built in Python.
#
# Run in the Isaac Sim Script Editor (File > Open, Ctrl+Enter) with the simulation stopped.
# It opens Lesson 2's robot (camera + 2D lidar), replaces the fixed speed of its drive graph
# with /cmd_vel, adds clock, odometry and TF publishers, adds a sensors graph for the camera
# and lidar, checks the wiring, and saves a NEW scene. Same result as lesson5_robot_ros_gui.usda.
#
# Node types and input/output names were checked in the 6.1 container (Nov 13-15), except
# IsaacCreateRenderProduct's camera input ("cameraPrim"): see section 5.
#
# PITFALLS (each one fails when run, or wires the wrong thing):
#   1. Commas between tuples in the node lists: ("a", "x") ("b", "y") without a comma is parsed
#      as CALLING the first tuple -> "TypeError: 'tuple' object is not callable". Python only
#      notices when the line runs, so the file still "parses".
#   2. Attribute paths have a COLON after inputs/outputs: "lin.inputs:tuple", never
#      "lin.inputs.tuple"; it's "inputs"/"outputs" (plural); names are case-sensitive.
#   3. Node names are matched exactly: a typo ("twsit") creates nothing to connect to.
#   4. The edit keys are keys.CREATE_NODES, keys.CONNECT, keys.SET_VALUES (no CREATE_NODE or
#      CREATE_CONNECTIONS).
#   5. The drive graph ALREADY exists in the Lesson 2 file with tick, diff and artic: creating
#      them again fails ("already exists"), and setting diff's fixed linearVelocity is pointless
#      once a wire feeds it. Only the new nodes and wires are added to that graph.
#   6. ROS2SubscribeTwist has no frameId input: it receives; frames belong to publishers.
#   7. type/topicName/frameId belong to the camera HELPER (img), not the render-product node.
#   8. Every render product must be told WHICH prim to render (camera, lidar), and each helper
#      needs its execIn wire (rp_lidar.execOut -> scan.execIn).
#   9. Short node names in edit() only resolve for nodes created in the SAME call; existing
#      nodes (tick, diff, artic in the drive graph) need full paths (seen on the first run:
#      "Failed to connect tick.outputs:tick -> ...").
#  10. staticPublisher = True changes only the QoS (kept for late subscribers), not the topic:
#      topicName defaults to "tf", so without topicName "tf_static" the sensor transforms land
#      on /tf and /tf_static doesn't exist (seen on the second run).
#  11. Every data input that matters must have exactly ONE wire; section 6 checks it before
#      saving (Part B's duplicate-wire bug).

import omni.graph.core as og
import omni.timeline
import omni.usd
from usdrt import Sdf as RtSdf             # OmniGraph relationship inputs take usdrt paths

# --- 1. Settings ------------------------------------------------------------------------
SOURCE = "/isaac-sim/work/lesson2_robot_with_graph_sensors_py.usda"   # Lesson 2 robot
OUT = "/isaac-sim/work/lesson5_robot_ros_py.usda"                      # new file
BODY = "/World/SimpleRobot/Cube"                    # chassis, articulation link
MOUNT = "/World/SimpleRobot/Front_Sensor"
CAMERA = "/World/SimpleRobot/Front_Sensor/Camera"   # translate must be (0.06, 0, 0)
LIDAR = "/World/SimpleRobot/Front_Sensor/Lidar"
DRIVE = "/World/DiffDriveGraph"                     # exists in SOURCE: tick, diff, artic
ROS = "/World/RosGraph"
SENSORS = "/World/SensorsGraph"

# --- 2. Guard + open the source scene ---------------------------------------------------
timeline = omni.timeline.get_timeline_interface()
if not timeline.is_stopped():                       # replacing a live stage crashed Kit
    timeline.stop()
    raise SystemExit("Simulation was running: stopped it. Run the script again.")

ctx = omni.usd.get_context()
if not ctx.open_stage(SOURCE):
    raise SystemExit(f"could not open {SOURCE}")
stage = ctx.get_stage()
for p in (BODY, MOUNT, CAMERA, LIDAR, DRIVE):       # fail fast on a wrong path
    if not stage.GetPrimAtPath(p).IsValid():
        raise SystemExit(f"missing prim {p}")

keys = og.Controller.Keys

# --- 3. Drive graph: /cmd_vel instead of the fixed speed (B1) ---------------------------
# Edit the EXISTING graph: add only the new nodes and wires.
# Short names ("twist") work only for nodes CREATED in this edit() call; the controller
# remembers those. Nodes that already exist in the file (tick, diff, artic) need FULL paths,
# or the connect fails ("Failed to connect tick.outputs:tick -> ...").
og.Controller.edit(
    DRIVE,
    {
        keys.CREATE_NODES: [
            ("twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),   # outputs: linearVelocity, angularVelocity
            ("lin", "omni.graph.nodes.BreakVector3"),               # outputs: x, y, z
            ("ang", "omni.graph.nodes.BreakVector3"),
        ],
        keys.CONNECT: [
            (f"{DRIVE}/tick.outputs:tick", "twist.inputs:execIn"),
            ("twist.outputs:linearVelocity", "lin.inputs:tuple"),
            ("twist.outputs:angularVelocity", "ang.inputs:tuple"),
            ("lin.outputs:x", f"{DRIVE}/diff.inputs:linearVelocity"),        # forward speed = linear.x
            ("ang.outputs:z", f"{DRIVE}/diff.inputs:angularVelocity"),       # turn rate = angular.z (+ = left)
        ],
        keys.SET_VALUES: [
            ("twist.inputs:topicName", "cmd_vel"),
        ],
    },
)

# --- 4. Clock, odometry, TF (B1-B2) -----------------------------------------------------
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
            ("odomc.inputs:chassisPrim", [RtSdf.Path(BODY)]),
            ("odom.inputs:odomFrameId", "World"),               # match the TF names
            ("odom.inputs:chassisFrameId", "Cube"),
            ("tf.inputs:parentPrim", [RtSdf.Path("/World")]),
            ("tf.inputs:targetPrims", [RtSdf.Path(BODY)]),       # articulation brings wheels + Front_Sensor
            ("tfs.inputs:parentPrim", [RtSdf.Path(MOUNT)]),
            ("tfs.inputs:targetPrims", [RtSdf.Path(CAMERA), RtSdf.Path(LIDAR)]),
            ("tfs.inputs:staticPublisher", True),                # static QoS (kept for late subscribers)...
            ("tfs.inputs:topicName", "tf_static"),               # ...but the TOPIC must be named too: default is "tf"
        ],
    },
)

# --- 5. Sensors graph (C1) --------------------------------------------------------------
# NOTE: the camera input of IsaacCreateRenderProduct is "cameraPrim" in Isaac Sim's own example
# scripts, but my offline check didn't list it. If this section errors on it, read the input's
# name in the node's Property panel and change both lines.
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
            ("rp_cam.inputs:cameraPrim", [RtSdf.Path(CAMERA)]),      # WHICH prim to render
            ("rp_cam.inputs:width", 640),                            # not swapped: fx = 554 px
            ("rp_cam.inputs:height", 480),
            ("rp_lidar.inputs:cameraPrim", [RtSdf.Path(LIDAR)]),
            ("img.inputs:type", "rgb"),
            ("img.inputs:topicName", "image_raw"),
            ("img.inputs:frameId", "Camera"),                        # = TF frame names
            ("info.inputs:topicName", "camera_info"),
            ("info.inputs:frameId", "Camera"),
            ("scan.inputs:type", "laser_scan"),
            ("scan.inputs:topicName", "scan"),
            ("scan.inputs:frameId", "Lidar"),
        ],
    },
)

# --- 6. Check the wiring before saving --------------------------------------------------
# Every data input below must have exactly ONE upstream wire. Zero = not connected (the node
# runs on a default); two or more = OmniGraph picks one, maybe the wrong one (Part B's bug).
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
    sources = og.Controller.attribute(path).get_upstream_connections()
    names = [s.get_path() for s in sources]
    status = "ok" if len(names) == 1 else "PROBLEM"
    print(f"{status:8s} {path} <- {names}")
    if len(names) != 1:
        problems.append(path)
if problems:
    raise SystemExit(f"{len(problems)} input(s) without exactly one wire: {problems} (scene NOT saved)")

# --- 7. Save to a NEW file --------------------------------------------------------------
# Export, never Save(): Save() would overwrite the Lesson 2 file this stage was opened from.
stage.GetRootLayer().Export(OUT)
print(f"graphs built and checked; saved {OUT}")

# --- 8. Test (with the ROS container) ----------------------------------------------------
# Play, then: ros2 topic list; ros2 topic echo /tf --once | grep frame_id;
# ros2 topic echo /tf_static --once | grep frame_id;
# ros2 topic echo /camera_info --once | grep -A1 "^k:"  (554); drive with
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
