# Unit 2 proof (Lesson 6), process 2 of 3: the CONTROLLER.
#
# A minimal ROS 2 node (rclpy), run in the ROS 2 container with use_sim_time:=true. It knows
# nothing about Isaac Sim: it reads /odom and publishes /cmd_vel, exactly as it would on the
# real robot. Drives straight along the aisle line to GOAL distance, stops, exits.
#
# Run (run_proof.sh does this; the file lives in /data/ros2/proof/, i.e. /data/proof/ inside
# the ROS 2 container):
#   python3 /data/proof/aisle_driver.py --ros-args -p use_sim_time:=true
#
# Everything below is standard ROS 2 Jazzy / rclpy (no Isaac Sim API).


# --- 1. Imports ---------------------------------------------------------------------------
# rclpy, rclpy.node.Node; geometry_msgs.msg Twist; nav_msgs.msg Odometry; math


# --- 2. Parameters (declare_parameter, so they can be changed with -p name:=value) ---------
# goal_distance  30.0  m along the aisle, measured from the START pose
# v_max          0.5   m/s forward
# k_lateral      1.0   rad/s per m of sideways error
# k_heading      1.5   rad/s per rad of heading error
# w_max          0.5   rad/s
# rate_hz        20.0  command rate (in SIM time: with use_sim_time the timer runs on /clock)
# odom_timeout   0.5   s (sim): no fresh /odom -> publish zero (the watchdog from Lesson 5)


# --- 3. The node class ----------------------------------------------------------------------
# __init__: publisher /cmd_vel (Twist), subscriber /odom (Odometry), timer at rate_hz.
# Odometry callback: store x, y, yaw (yaw from the quaternion: atan2(2(wz+xy), 1-2(y^2+z^2)))
#                    and the time it arrived (self.get_clock().now(): SIM time here).
#   The FIRST message defines the start pose: the aisle line is the start heading through the
#   start point. Work in that frame, so it doesn't matter whether /odom starts at (0, 0) or
#   at the spawn position.                                               [check which on the VM]


# --- 4. Control law (timer callback) -----------------------------------------------------------
# No odom yet, or older than odom_timeout  -> publish zero, return (watchdog)
# In the start frame: along = distance travelled along the line, lateral = sideways offset,
#                     heading_err = yaw - start_yaw (wrapped to -pi..pi)
# if along >= goal_distance: publish zero a few times, log "goal reached", shut down
# v = v_max, slowing linearly over the last 2 m (avoid overshoot)
# w = -k_lateral * lateral - k_heading * heading_err, clamped to +-w_max
#   (+w turns left: ROS convention; a robot left of the line (lateral > 0) must turn right)
# publish Twist(linear.x = v, angular.z = w)
# Log every second: along, lateral, heading_err, v, w


# --- 5. main(): rclpy.init, spin, clean shutdown (always send a final zero command) -----------
