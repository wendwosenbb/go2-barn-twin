#!/usr/bin/env bash
# Unit 2 proof (Lesson 6), process 3 of 3 + orchestration: ONE command for the whole proof.
#
# Runs on the VM:
#   ssh ubuntu@isaac-sim-ws 'bash /data/isaac-sim/work/run_proof.sh'
#   1. check preconditions (containers up, files present, nobody else publishing /clock)
#   2. start the rosbag recorder      (ROS 2 container, background, sim time)
#   3. start the controller           (ROS 2 container, background, sim time)
#   4. run the simulator headless     (Isaac Sim container, foreground; decides PASS/FAIL)
#   5. stop recorder and controller cleanly (SIGINT, so the bag is closed properly)
#   6. check the bag has every topic, print report + bag summary + controller log
#   7. exit 0 only if the simulator's report says PASS AND the bag check passes
#
# Containers: isaac-sim (infra/vm/isaac-run.sh), ros2 (Lesson 5: --network host, UDPv4
# transport, /data/ros2 mounted at /data). Copy the files from the laptop first:
#   scp isaac_sim/unit2/lesson6_proof/{proof_sim.py,run_proof.sh} ubuntu@isaac-sim-ws:/data/isaac-sim/work/
#   ssh ubuntu@isaac-sim-ws 'mkdir -p /data/ros2/proof'
#   scp isaac_sim/unit2/lesson6_proof/aisle_driver.py ubuntu@isaac-sim-ws:/data/ros2/proof/
#
# Short test run: GOAL=5 bash run_proof.sh   (controller goal distance in m; the simulator's
# GOAL_X stays 30, so that run must report FAIL on "reached goal x": a good check that FAIL works)
#
# Not verified yet: `ros2 bag record --use-sim-time` (Jazzy option; stamps the bag with /clock
# time, so a replay's timeline is sim time).
set -euo pipefail

# --- 1. Settings --------------------------------------------------------------------------
RUN_ID=$(date +%Y%m%d_%H%M%S)          # one bag + log per run, never overwritten
BAG=/data/bags/proof_$RUN_ID           # path INSIDE the ros2 container (= /data/ros2/bags/ on the VM)
PROOF=/data/proof                      # inside ros2 (= /data/ros2/proof/ on the VM)
DRIVER_LOG=$PROOF/driver_$RUN_ID.log
BAG_PID=$PROOF/bag.pid                 # PID files: stop processes with the bash builtin `kill`,
DRIVER_PID=$PROOF/driver.pid           # no dependence on pkill being in the image
WORK=/data/isaac-sim/work              # on the VM (= /isaac-sim/work inside isaac-sim)
REPORT=$WORK/proof_report.txt
GOAL=${GOAL:-30.0}                     # controller goal distance, override from the environment
TOPICS=(/clock /tf /tf_static /odom /cmd_vel /scan /camera_info /image_raw)
ROS="source /opt/ros/jazzy/setup.bash"

die() { echo "ERROR: $*" >&2; exit 2; }
info() { echo "--- $*"; }
ros() { docker exec ros2 bash -c "$ROS && $1"; }   # run one command in the ROS 2 container


# --- 2. Preconditions (fail fast, with a clear message) --------------------------------------
for c in isaac-sim ros2; do
  [[ "$(docker inspect -f '{{.State.Running}}' "$c" 2>/dev/null)" == "true" ]] \
    || die "container $c is not running (isaac-run / docker start ros2)"
done
[[ -f $WORK/proof_sim.py ]] || die "missing $WORK/proof_sim.py"
[[ -f /data/ros2/proof/aisle_driver.py ]] || die "missing /data/ros2/proof/aisle_driver.py"
# Somebody already publishing /clock = the streamed GUI is playing a ROS scene. Two simulations
# on the same topics would mix (proof_sim.py header). Stop the GUI's simulation first.
if ros "timeout 4 ros2 topic echo /clock --once" >/dev/null 2>&1; then
  die "/clock is already being published: stop the simulation in the GUI first"
fi
info "preconditions ok (run $RUN_ID, goal $GOAL m)"


# --- 3. Clean-up on any exit ------------------------------------------------------------------
# SIGINT (not SIGKILL): ros2 bag record then closes the file properly; a killed recorder leaves
# a bag without its index. Then wait until the recorder has really exited.
stop_ros_side() {
  ros "for f in $DRIVER_PID $BAG_PID; do [ -f \$f ] && kill -INT \$(cat \$f) 2>/dev/null; done; \
       for i in \$(seq 1 40); do [ -f $BAG_PID ] && kill -0 \$(cat $BAG_PID) 2>/dev/null || break; sleep 0.5; done; \
       rm -f $DRIVER_PID $BAG_PID" || true
}
trap stop_ros_side EXIT


# --- 4. Start the recorder, then the controller (both wait for /clock) ---------------------------
# `echo $$ > file; exec cmd`: the shell writes its PID, then BECOMES the command (same PID),
# so the PID file points at the recorder/controller itself.
docker exec -d ros2 bash -c "$ROS && echo \$\$ > $BAG_PID && \
  exec ros2 bag record --use-sim-time -o $BAG ${TOPICS[*]} > $PROOF/bag_$RUN_ID.log 2>&1"
docker exec -d ros2 bash -c "$ROS && echo \$\$ > $DRIVER_PID && \
  exec python3 $PROOF/aisle_driver.py --ros-args -p use_sim_time:=true -p goal_distance:=$GOAL \
  > $DRIVER_LOG 2>&1"
sleep 5                                # discovery, and both processes up
ros "kill -0 \$(cat $BAG_PID)" 2>/dev/null || die "recorder didn't start: see /data/ros2/proof/bag_$RUN_ID.log"
ros "kill -0 \$(cat $DRIVER_PID)" 2>/dev/null || die "controller didn't start: see /data/ros2/proof/driver_$RUN_ID.log"
info "recorder -> $BAG, controller running"


# --- 5. Run the simulator (foreground), keep its exit code ---------------------------------------
rm -f "$REPORT"                        # a stale report from an earlier run must not count
info "simulator starting (headless; loading takes a minute or two)"
set +e
docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/proof_sim.py
SIM_RC=$?
set -e
info "simulator exited with code $SIM_RC"


# --- 6. Stop recorder and controller now (the trap would too, but we need the bag closed) --------
stop_ros_side
trap - EXIT


# --- 7. Summary and verdict ------------------------------------------------------------------------
echo
info "simulator report"
if [[ -f $REPORT ]]; then cat "$REPORT"; else echo "(no report: the simulator failed before writing it)"; fi

echo
info "controller log (last 8 lines)"
docker exec ros2 tail -n 8 "$DRIVER_LOG" || true

echo
info "bag"
BAG_INFO=$(ros "ros2 bag info $BAG" 2>&1) || BAG_INFO="(ros2 bag info failed)"
echo "$BAG_INFO"
BAG_OK=1
for t in "${TOPICS[@]}"; do
  # A topic line looks like: "Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 3725 | ..."
  count=$(echo "$BAG_INFO" | grep -E "Topic: $t \|" | sed -E 's/.*Count: ([0-9]+).*/\1/' || true)
  if [[ -z $count || $count -eq 0 ]]; then
    echo "bag check: $t MISSING or empty"; BAG_OK=0
  fi
done
[[ $BAG_OK -eq 1 ]] && echo "bag check: all ${#TOPICS[@]} topics recorded"

echo
# The report line is the reliable signal (proof_sim.py header, improvement c); the exit code
# is shown as a cross-check.
if [[ -f $REPORT ]] && grep -q "^RESULT: PASS" "$REPORT" && [[ $BAG_OK -eq 1 ]]; then
  echo "UNIT 2 PROOF: PASS   (run $RUN_ID, bag /data/ros2/bags/proof_$RUN_ID, sim exit $SIM_RC)"
  exit 0
fi
echo "UNIT 2 PROOF: FAIL   (run $RUN_ID, sim exit $SIM_RC, bag check $BAG_OK)"
exit 1
