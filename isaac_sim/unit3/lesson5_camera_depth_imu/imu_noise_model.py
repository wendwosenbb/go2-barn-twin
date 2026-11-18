# Unit 3, Lesson 5 (laptop part): the IMU noise model, and how it's measured (Allan deviation).
#
# Both the Go2 and the Mid-360 carry an IMU; legged lidar SLAM (FAST-LIO) leans on it to
# undo motion distortion (Lesson 1, lidar_raycast_toy.py). A simulated IMU that's perfect makes
# SLAM look better in sim than on the robot. The standard model per axis:
#
#   measured = true + bias(t) + white noise
#   white noise      N(0, sigma_w^2 / dt)     "noise density"     (gyro: rad/s/sqrt(Hz))
#   bias(t)          random walk: bias += N(0, sigma_b^2 * dt)   "bias instability / walk"
#
# The Allan deviation plot separates them: slope -1/2 at short averaging times = white noise;
# slope +1/2 at long times = bias random walk; the flat bottom = bias instability. Datasheets
# give these numbers; Isaac Sim's IMU sensor has noise settings to match them ([UNVERIFIED]
# names in 6.1, Lesson 5 GPU part).
#
# Status: OUTLINE. Steps to write (all numpy, laptop):
#   1. simulate a stationary gyro at 200 Hz for 2 hours with sigma_w and sigma_b of your choice
#   2. integrate: heading drift over 1 min, 10 min (why SLAM needs the lidar, not the IMU alone)
#   3. Allan deviation (overlapping), log-log plot; read sigma_w back at tau = 1 s
#   4. later: the same on a real stationary Go2/Mid-360 IMU log (rosbag), and on the sim IMU
#
# Camera and depth (GPU part of the lesson): intrinsics and square pixels (Unit 2 Lesson 5),
# depth noise growing with distance (stereo depth error ~ z^2), rolling shutter, exposure.
