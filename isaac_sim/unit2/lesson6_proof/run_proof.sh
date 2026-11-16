#!/usr/bin/env bash
# Unit 2 proof (Lesson 6), process 3 of 3 + orchestration: ONE command for the whole proof.
#
# Runs on the VM (ssh ubuntu@isaac-sim-ws 'bash /data/isaac-sim/work/run_proof.sh'):
#   1. start the rosbag recorder      (ROS 2 container, background)
#   2. start the controller           (ROS 2 container, background, sim time)
#   3. run the simulator headless     (Isaac Sim container, foreground; decides PASS/FAIL)
#   4. stop recorder and controller cleanly, print the report and the bag summary
#   5. exit with the simulator's code (0 = PASS)
#
# Containers: isaac-sim (from isaac-run), ros2 (Lesson 5: --network host, UDPv4 transport,
# /data/ros2 mounted at /data). Files: proof_sim.py in /data/isaac-sim/work/,
# aisle_driver.py in /data/ros2/proof/.
set -euo pipefail

# --- 1. Settings --------------------------------------------------------------------------
# RUN_ID=$(date +%Y%m%d_%H%M%S)        one bag + report per run, never overwritten
# BAG=/data/bags/proof_$RUN_ID          path INSIDE the ros2 container
# TOPICS: /clock /tf /tf_static /odom /cmd_vel /scan /camera_info /image_raw
# ROS="source /opt/ros/jazzy/setup.bash"


# --- 2. Preconditions (fail fast, with a clear message) --------------------------------------
# both containers running (docker ps --filter name=...)
# the Isaac Sim app is loaded ("Full Streaming App is loaded" in docker logs)? Note: the
#   streamed GUI app and the headless proof are two Kit processes on one GPU (Lesson 4 ran
#   alongside it fine). Decide: keep the GUI running (to watch) or stop it (more GPU for the
#   proof, higher RTF).
# files present: proof_sim.py, aisle_driver.py


# --- 3. Clean-up on exit ---------------------------------------------------------------------
# trap: on any exit, stop the recorder with SIGINT (so the bag is closed properly, not
#   corrupted) and stop the controller:
#   docker exec ros2 pkill -INT -f "ros2 bag record"
#   docker exec ros2 pkill -INT -f aisle_driver.py


# --- 4. Start the recorder, then the controller (both wait for /clock) ---------------------------
# docker exec -d ros2 bash -c "$ROS && ros2 bag record -o $BAG <topics>"
# docker exec -d ros2 bash -c "$ROS && python3 /data/proof/aisle_driver.py --ros-args -p use_sim_time:=true > /data/proof/driver_$RUN_ID.log 2>&1"
# sleep a few seconds: discovery


# --- 5. Run the simulator (foreground), keep its exit code ---------------------------------------
# set +e; docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/proof_sim.py; SIM_RC=$?; set -e


# --- 6. Stop the recorder (trap does it), give it a moment to close the file ---------------------


# --- 7. Summary ---------------------------------------------------------------------------------
# cat the report (proof_sim.py writes it under /data/isaac-sim/work/)
# docker exec ros2 bash -c "$ROS && ros2 bag info $BAG"
# tail the controller log
# echo PASS/FAIL from SIM_RC; exit $SIM_RC
