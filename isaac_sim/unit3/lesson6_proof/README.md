# Unit 3 proof: L2 and Mid-360 sensor twins (outline)

Goal: the robot in the barn carries simulated twins of its real lidars, and robot software
accepts their topics without changes.

1. **Mid-360 twin** (Lesson 3), validated against the datasheet: FOV, points per frame,
   coverage growth, minimum range, per-point time; publishes the same fields as
   `livox_ros_driver2`.
2. **Unitree L2 twin**: collect the datasheet first (FOV, point rate, scan pattern,
   repetitive or not), then the same validation as the Mid-360.
3. **Realism report** (Lesson 4): fitted beam-model parameters of each sim lidar, side by
   side with the datasheet. Later (Unit 4): with real barn logs.
4. **Run**: the Unit 2 proof pipeline (`run_proof.sh`) with these sensors on board; a bag
   with both point clouds + IMU; FAST-LIO (or the robot's SLAM) builds a map from the sim bag.

Pass criteria are written before the run, as in Unit 2 Lesson 6.
