# Unit 2, Lesson 6 experiment: does timeCodesPerSecond (or framesPerSecond) set the FRAME STEP?
#
# Question: with timeCodesPerSecond = 1, does the app advance 1 s of sim time per frame, so that
# graphs tick and sensors sample once per simulated second? Or does it only relabel the
# timeline's ruler?
#
# Method: open the barn (no robot, no ROS), change ONE stage setting in memory (never saved),
# Play, and measure per app frame: sim time added (the frame step), physics steps, wall time.
# Predictions written before running (Nov 18): graphs once per sim second, physics unchanged,
# RTF = 1.
#
# Run (VM, headless), one case per run so every start is clean:
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/timecode_experiment.py base
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/timecode_experiment.py tcps1
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/timecode_experiment.py fps1
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/timecode_experiment.py tl_fps1
#   docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/timecode_experiment.py rm_dt1
import sys

from isaacsim import SimulationApp

CASE = sys.argv[1] if len(sys.argv) > 1 else "base"
simulation_app = SimulationApp({"headless": True})

try:
    import time

    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.timeline
    from isaacsim.core.simulation_manager import SimulationManager

    BARN = "/isaac-sim/work/barn_usd/barns/farm1_barn1/barn_sim.usda"
    FRAMES = 20

    ok, stage = stage_utils.open_stage(BARN)
    if not ok:
        raise SystemExit(f"could not open {BARN}")
    print(f"[{CASE}] stage as authored: timeCodesPerSecond {stage.GetTimeCodesPerSecond()}, "
          f"framesPerSecond {stage.GetFramesPerSecond()}")
    if CASE == "tcps1":
        stage.SetTimeCodesPerSecond(1.0)          # in memory only: the file is never saved
    elif CASE == "fps1":
        stage.SetFramesPerSecond(1.0)
    elif CASE not in ("base", "tl_fps1", "rm_dt1"):
        raise SystemExit(f"unknown case {CASE}: base, tcps1, fps1, tl_fps1 or rm_dt1")
    simulation_app.update()                       # let the app/timeline see the change
    tl = omni.timeline.get_timeline_interface()
    if CASE == "tl_fps1":                         # the TIMELINE's rate, not the stage's
        tl.set_target_framerate(1.0)              # [UNVERIFIED] name; printed back below
        simulation_app.update()
    from isaacsim.core.rendering_manager import RenderingManager   # the app's frame step
    if CASE == "rm_dt1":
        RenderingManager.set_dt(1.0)              # 1 s of sim time per app frame
        simulation_app.update()
    print(f"[{CASE}] RenderingManager.get_dt(): {RenderingManager.get_dt()}")
    print(f"[{CASE}] stage now: timeCodesPerSecond {stage.GetTimeCodesPerSecond()}, "
          f"framesPerSecond {stage.GetFramesPerSecond()}")
    for name in ("get_time_codes_per_seconds", "get_target_framerate"):   # [UNVERIFIED] names
        f = getattr(tl, name, None)
        print(f"[{CASE}] timeline.{name}(): {f() if f else 'not available'}")

    app_utils.play()
    simulation_app.update()                       # process Play
    dt_phys = SimulationManager.get_physics_dt()

    rows = []
    for _ in range(FRAMES):
        s0, w0 = SimulationManager.get_simulation_time(), time.time()
        tl0 = tl.get_current_time()               # timeline time, seconds
        simulation_app.update()                   # ONE app frame
        rows.append((SimulationManager.get_simulation_time() - s0, time.time() - w0,
                     tl.get_current_time() - tl0))
    sim = [r[0] for r in rows[5:]]                # skip the first frames (warm-up)
    wall = [r[1] for r in rows[5:]]
    tline = [r[2] for r in rows[5:]]
    step = sum(sim) / len(sim)
    print(f"[{CASE}] physics dt {dt_phys:.4f} s")
    print(f"[{CASE}] frame step (sim time per frame)  {step:.5f} s  = 1/{1 / step:.2f}")
    print(f"[{CASE}] timeline time per frame          {sum(tline) / len(tline):.5f} s")
    print(f"[{CASE}] physics steps per frame          {step / dt_phys:.2f}")
    print(f"[{CASE}] wall time per frame              {sum(wall) / len(wall):.4f} s")
    print(f"[{CASE}] RTF                              {sum(sim) / sum(wall):.2f}")
    app_utils.stop()

finally:
    simulation_app.close()
