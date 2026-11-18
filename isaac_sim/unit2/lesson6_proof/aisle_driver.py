# Unit 2 proof (Lesson 6), process 2 of 3: the CONTROLLER.
#
# A minimal ROS 2 node (rclpy), run in the ROS 2 container with use_sim_time:=true. It knows
# nothing about Isaac Sim: it reads /odom and publishes /cmd_vel, exactly as it would on the
# real robot. Drives straight along the aisle line to GOAL distance, stops, exits.
#
# Run (run_proof.sh does this; the file lives in /data/ros2/proof/, i.e. /data/proof/ inside
# the ROS 2 container):
#   python3 /data/proof/aisle_driver.py --ros-args -p use_sim_time:=true
#   (change any parameter the same way, e.g. -p goal_distance:=5.0 for a short test)
#
# Everything below is standard ROS 2 Jazzy / rclpy (no Isaac Sim API).
#
# Pitfalls:
#  1. rclpy.init / spin / shutdown live in the package itself: `import rclpy`, not only
#     rclpy.node.
#  2. `super().__init__(...)` with parentheses: super is a function; `super.__init__` looks up
#     __init__ on the built-in `super` type and fails.
#  3. A publisher is an object: `self.pub_command.publish(cmd)`. Calling it like a function
#     raises "'Publisher' object is not callable".
#  4. `now - latest_time` is an rclpy Duration; comparing it with a float raises TypeError.
#     Convert first: `(now - t).nanoseconds * 1e-9`.
#  5. `is None`, not `== None` (identity check; the Python idiom).
#  6. Tunable numbers are ROS parameters (declare_parameter), so a run can change them with
#     -p name:=value instead of editing the code.
#  7. Naming: Python classes use CapWords (AisleDriver); ROS node names use snake_case
#     ("aisle_driver"), matching the file name.
#
# Improvements:
#  a. The first /odom message defines the START pose; all errors are computed in that frame,
#     so it doesn't matter whether Isaac's odometry starts at (0, 0) or at the spawn point.
#  b. The control law is a pure function (no ROS inside): easy to read and to test off-line.
#  c. Stop sequence: after the goal, zero commands for 1 s (sim), then the node exits by
#     itself, so run_proof.sh doesn't have to kill it.
#  d. A final zero command on Ctrl+C / SIGINT, sent best-effort (see main()).

import math                                     # atan2, sin, cos

import rclpy                                    # init / spin / shutdown live HERE
from rclpy.node import Node                     # base class of every rclpy node

from geometry_msgs.msg import Twist             # /cmd_vel: linear.x (m/s), angular.z (rad/s)
from nav_msgs.msg import Odometry               # /odom: pose (position + quaternion) + twist


# --- Control law: pure maths, no ROS (improvement b) ------------------------------------------
def wrap_angle(a):
    """Wrap an angle to -pi..pi, so +350 deg becomes -10 deg (the short way round)."""
    return math.atan2(math.sin(a), math.cos(a))


def to_start_frame(x, y, yaw, x0, y0, yaw0):
    """Express the robot pose in the START frame: x axis = start heading, origin = start point.

    along   : distance travelled along the line (what the goal is measured in)
    lateral : sideways offset from the line, + = robot is LEFT of it (ROS: y is left)
    heading : yaw error relative to the line, + = robot points to the left of it
    """
    dx, dy = x - x0, y - y0                     # displacement in the odom frame
    c, s = math.cos(yaw0), math.sin(yaw0)       # rotate it by -yaw0 into the start frame
    along = c * dx + s * dy
    lateral = -s * dx + c * dy
    heading = wrap_angle(yaw - yaw0)
    return along, lateral, heading


def control(along, lateral, heading, p):
    """Return (v, w): forward speed and turn rate for one control step.

    v: v_max, slowing linearly over the last slow_down m so it doesn't overshoot the goal;
       never below v_min, or the robot would creep towards the goal forever.
    w: proportional steering back onto the line. +w turns LEFT (ROS convention), so a robot
       left of the line (lateral > 0) or pointing left (heading > 0) gets a NEGATIVE w.
    """
    remaining = p["goal_distance"] - along
    v = p["v_max"] * min(1.0, remaining / p["slow_down"])
    v = max(p["v_min"], v)
    w = -p["k_lateral"] * lateral - p["k_heading"] * heading
    w = max(-p["w_max"], min(p["w_max"], w))    # clamp to +-w_max
    return v, w


# --- The node -----------------------------------------------------------------------------
class AisleDriver(Node):                        # classes: CapWords

    def __init__(self):
        super().__init__("aisle_driver")        # super() with (); node name in snake_case

        # Parameters: defaults here, override with --ros-args -p name:=value.
        # use_sim_time is NOT declared here: every node already has it built in.
        defaults = {
            "goal_distance": 30.0,              # m along the aisle, from the start pose
            "v_max": 0.5,                       # m/s cruising speed
            "v_min": 0.05,                      # m/s floor while slowing down
            "slow_down": 2.0,                   # m before the goal where slowing starts
            "k_lateral": 1.0,                   # rad/s per m of sideways error
            "k_heading": 1.5,                   # rad/s per rad of heading error
            "w_max": 0.5,                       # rad/s turn-rate limit
            "rate_hz": 20.0,                    # command rate, in SIM time (see below)
            "odom_timeout": 0.5,                # s (sim) without fresh /odom -> stop
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.pub_command = self.create_publisher(Twist, "/cmd_vel", 10)
        # Depth 1: only the newest odometry matters; an old one is useless for control.
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 1)

        # With use_sim_time the timer runs on /clock: 20 Hz of SIM time, whatever the RTF.
        # Before the first /clock message sim time is 0 and the timer doesn't fire at all.
        self.timer = self.create_timer(1.0 / self.p["rate_hz"], self.timer_callback)

        self.x = None                           # latest pose from /odom (None = none yet)
        self.y = None
        self.yaw = None
        self.latest_time = None                 # when it arrived, node clock (= sim time)
        self.start = None                       # (x0, y0, yaw0): first pose (improvement a)
        self.stop_ticks = None                  # counts zero commands after the goal
        self.done = False                       # main() exits when True
        self.last_log = None                    # sim time of the last progress line

        self.get_logger().info(f"aisle_driver ready: {self.p}")

    def odom_callback(self, msg: Odometry):
        self.latest_time = self.get_clock().now()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Quaternion -> yaw (rotation about Z); exact for a robot that stays level.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.start is None:                  # improvement a
            self.start = (self.x, self.y, self.yaw)
            self.get_logger().info(
                f"start pose x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f} deg")

    def publish(self, v, w):
        cmd = Twist()                           # all fields start at 0.0
        cmd.linear.x = float(v)                 # float(): ROS message fields reject ints
        cmd.angular.z = float(w)
        self.pub_command.publish(cmd)           # a publisher is an object: .publish()

    def timer_callback(self):
        now = self.get_clock().now()

        # Watchdog: no odometry yet, or too old -> stop (the Lesson 5 lesson: silence must
        # never mean "keep going").
        if self.x is None:                      # `is None`: identity, not ==
            self.publish(0.0, 0.0)
            return
        age = (now - self.latest_time).nanoseconds * 1e-9   # Duration -> float seconds
        if age > self.p["odom_timeout"]:
            self.publish(0.0, 0.0)
            self.get_logger().warn(f"/odom is {age:.2f} s old: stopping",
                                   throttle_duration_sec=1.0)
            return

        # Goal reached: send zeros for 1 s (sim) so the robot really stops, then finish.
        if self.stop_ticks is not None:
            self.publish(0.0, 0.0)
            self.stop_ticks += 1
            if self.stop_ticks >= self.p["rate_hz"]:
                self.done = True
            return

        along, lateral, heading = to_start_frame(self.x, self.y, self.yaw, *self.start)
        if along >= self.p["goal_distance"]:
            self.get_logger().info(
                f"goal reached: along={along:.2f} m lateral={lateral:+.3f} m")
            self.stop_ticks = 0
            self.publish(0.0, 0.0)
            return

        v, w = control(along, lateral, heading, self.p)
        self.publish(v, w)

        # Progress once per SIM second (throttle_duration_sec would count wall time).
        if self.last_log is None or (now - self.last_log).nanoseconds >= 1e9:
            self.last_log = now
            self.get_logger().info(
                f"t={now.nanoseconds * 1e-9:7.2f} along={along:6.2f} lateral={lateral:+.3f} "
                f"heading={math.degrees(heading):+5.1f} deg v={v:.2f} w={w:+.2f}")


def main():
    rclpy.init()
    node = AisleDriver()
    try:
        # spin_once in a loop instead of spin(): lets the node end itself when done.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:                   # Ctrl+C, or SIGINT from run_proof.sh
        pass
    finally:
        # Final zero command (improvement d). Best-effort: on SIGINT rclpy may already have
        # shut its context down, and then publishing fails. That's why the simulator side
        # must not rely on it either (the drive graph holds the LAST command).
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
