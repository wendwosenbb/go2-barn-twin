# Display helpers for an opened scene (runtime only, nothing saved).
import omni.usd
from isaacsim.sensors.experimental.rtx import LidarSensor
from omni.kit.viewport.utility import create_viewport_window, get_viewport_from_window_name

stage = omni.usd.get_context().get_stage()

# Find every lidar actually in the scene: never pass a path that might not exist.
lidars = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName() == "OmniLidar"]
print("lidars found:", lidars)
lidar_views = []                                   # top-level variable: keeps the sensors alive
for path in lidars:
    view = LidarSensor(path)
    view.attach_writer("RtxSensorDebugDrawPointCloud")
    lidar_views.append(view)                       # without this, Python deletes it and the writer goes too

CAMERA = "/World/SimpleRobot/Front_Sensor/Camera"                      # same name in both files
if stage.GetPrimAtPath(CAMERA).IsValid():
    vp = get_viewport_from_window_name("Robot Camera")                 # reuse the window if open
    if vp is None:
        vp = create_viewport_window("Robot Camera", width=640, height=360).viewport_api
    vp.camera_path = CAMERA
else:
    print("no camera at", CAMERA)

