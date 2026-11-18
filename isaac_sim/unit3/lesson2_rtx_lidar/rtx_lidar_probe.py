# Unit 3, Lesson 2 (GPU): how Isaac Sim's RTX lidar works, measured from the outside.
#
# Goal: treat the simulator's lidar like a real sensor on a test bench. Put a REPETITIVE lidar
# (Ouster OS1, shipped with Isaac Sim) in the barn, capture ONE frame of raw returns with
# per-point azimuth, elevation, range and time, save them, and analyse them on the laptop with
# Lesson 1's tools (scan_patterns.coverage, the beam model). The question each check answers:
# "does the sim produce what the datasheet and Lesson 1's theory say it should?"
#
# Run (VM, headless):
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/rtx_lidar_probe.py
# Output: /isaac-sim/work/unit3/os1_frame.npz  -> copy to the laptop for analysis
#
# Marks: [VERIFIED] checked in the 6.1 container in Unit 2; [UNVERIFIED] check first.
# Status: OUTLINE + the parts that are known. Fill in the [UNVERIFIED] steps on the VM.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})                          # [VERIFIED]

try:
    import numpy as np

    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.sensors.experimental.rtx import Lidar                     # [VERIFIED, U2 L2]
    from isaacsim.storage.native import get_assets_root_path                # [VERIFIED, U2 L2]
    from pxr import Gf, UsdGeom

    BARN = "/isaac-sim/work/barn_usd/barns/farm1_barn1/barn_sim.usda"
    OUT = "/isaac-sim/work/unit3/os1_frame.npz"
    MOUNT_POSE = (3.0, 0.31, 0.40)       # same spot and height as lidar_raycast_toy.py

    # --- 1. Which lidar profiles ship with 6.1? ------------------------------------------
    # The 6.0 docs mention a SUPPORTED_LIDAR_CONFIGS table in the experimental rtx extension.
    # [UNVERIFIED] import it and print the names; pick the OS1 one (e.g. a name like
    # "Ouster_OS1_REV6_128ch10hz1024res" in 5.x forum posts) and note the asset path.
    # from isaacsim.sensors.experimental.rtx import SUPPORTED_LIDAR_CONFIGS
    # for name in sorted(SUPPORTED_LIDAR_CONFIGS): print(name)
    OS1_ASSET = None                     # fill in from the printout, relative to the assets root

    # --- 2. Open the barn, place the lidar on a static stand ---------------------------
    ok, stage = stage_utils.open_stage(BARN)                                 # [VERIFIED]
    if not ok:
        raise SystemExit(f"could not open {BARN}")
    stand = UsdGeom.Xform.Define(stage, "/World/LidarStand")
    UsdGeom.XformCommonAPI(stand).SetTranslate(Gf.Vec3d(*MOUNT_POSE))
    lidar_path = "/World/LidarStand/OS1"
    prim = stage.DefinePrim(lidar_path, "OmniLidar")                         # [VERIFIED pattern]
    # prim.GetReferences().AddReference(get_assets_root_path() + OS1_ASSET)
    # lidar = Lidar(lidar_path)                                              # keep alive

    # --- 3. Read the profile like a datasheet ----------------------------------------------
    # [UNVERIFIED] list the OmniLidar prim's attributes (prim.GetAttributes()) and find:
    #   rotation rate, number of channels and their elevations, azimuth resolution,
    #   min/max range, range accuracy/noise settings, return mode, per-point time and ring
    #   (5.x forum: per-point time and ring for Ouster are opt-in, off by default!)
    # Write them into the report BEFORE capturing: those are your expected values.

    # --- 4. Capture one frame of raw returns ----------------------------------------------
    # [UNVERIFIED] attach an annotator that gives per-point azimuth, elevation, range,
    # timestamp (not only x, y, z). Candidates in older releases:
    # "IsaacExtractRTXSensorPointCloudNoAccumulator", "IsaacCreateRTXLidarScanBuffer".
    # Play, update a few frames until a FULL revolution is in the buffer, then read it.
    # np.savez(OUT, azimuth=..., elevation=..., range=..., time=..., intensity=...)

    # --- 5. Checks to run on the laptop (write them as analyse_frame.py) ----------------------
    # a. Elevations: a histogram should show exactly N_channels spikes at the datasheet angles.
    # b. Points per frame = channels x columns (128 x 1024 for a 1024-res 10 Hz profile).
    # c. Coverage vs integration time (scan_patterns.coverage): FLAT after one revolution.
    # d. Time: per-point times span ~0.1 s and increase with azimuth (one revolution).
    # e. Ranges against the barn geometry: wall hits at y = 4.78 / -8.02 (lidar_raycast_toy.cast
    #    gives the expected z* for the same beams): range error histogram -> beam model sigma.

    print("outline only: fill in the [UNVERIFIED] steps")

finally:
    simulation_app.close()
