# Unit 2, Lesson 4, B2: the Go2 standing in the barn, headless.
#
# Opens the barn (barn_sim.usda: shell, equipment, scan, lights, physics layer at 200 Hz),
# places the rotor-free Go2 from Lesson 3 at the mapping start spot, starts it in its
# standing pose, steps 5 s of physics itself, logs base height and joint errors to a report,
# saves the scene for a GUI check, and exits.
#
# Run (after copying to /data/isaac-sim/work/):
#   ssh ubuntu@isaac-sim-ws 'docker exec isaac-sim /isaac-sim/python.sh /isaac-sim/work/lesson4_go2_barn.py'
#
# Calls below marked "verified" were checked in the 6.1 container (source or examples).
#
# CORRECTIONS to the first draft (each would have stopped the run or given wrong results):
#   1. SimulationApp({{'headless': True}}) -> {"headless": True}: double braces make a SET
#      containing a dict, which errors.
#   2. stage = open_stage(BARN) -> ok, stage = open_stage(BARN): it returns (bool, stage);
#      check ok and fail fast.
#   3. stage.traverse() -> stage.Traverse() (USD methods are capitalised), and the scene check
#      compared the path to 'World/PhysicsScene' (no leading "/", and a Path vs a str): it
#      never matched, so the count was always 0. Check the TYPE: prim.IsA(UsdPhysics.Scene).
#   4. SimulationManager.play() / .update() don't exist -> app_utils.play() and
#      simulation_app.update().
#   5. RigidPrim on the base has no joints (no dof_names, no get_dof_positions) -> a robot is
#      read with Articulation, created AFTER play (it exists in the physics engine only then).
#   6. The loop stepped but measured nothing; no report, no saved scene.
#
# IMPROVEMENTS:
#   a. Settings (paths, spawn, pose, duration, log rate) in one place at the top.
#   b. The articulation root is FOUND (prim with ArticulationRootAPI under /World/Go2), not
#      guessed: one less path that can silently be wrong.
#   c. Joint readings come in the articulation's own order (dof_names) and in RADIANS; targets
#      are reordered by name and errors computed in degrees, so they compare like with like.
#   d. Steps and log interval are derived from the physics dt; times are reported relative to
#      the loop's start (play() + the first update already advanced physics, Part A).
#   e. Every message goes to both the console and the report (log()); the report is written
#      before saving the scene, so a failed save doesn't lose the results.
#   f. Stop before saving, so the saved scene holds the authored start state, not a mid-run
#      snapshot; save_stage writes a NEW file and fixes relative paths (barn_sim.usda untouched).


# --- 1. Start the app (FIRST, before any isaacsim/omni import) ----------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})          # a dict: single braces

# --- 2. Settings in one place -----------------------------------------------------------
BARN = "/isaac-sim/work/barn_usd/barns/farm1_barn1/barn_sim.usda"
GO2 = "/isaac-sim/work/go2_py/go2_description_norotor/go2_description_norotor.usda"
REPORT = "/isaac-sim/work/lesson4_go2_barn_report.txt"
SAVED_SCENE = "/isaac-sim/work/lesson4_go2_barn_scene.usda"  # a NEW file (see section 12)
SPAWN = (0.0, 0.0, 0.35)          # barn origin = mapping start spot; floor top at z = 0
DURATION_S = 5.0
LOG_EVERY_S = 0.1
POSE = {"FL_hip_joint": 5.7, "RL_hip_joint": 5.7, "FR_hip_joint": -5.7, "RR_hip_joint": -5.7,
        "FL_thigh_joint": 45.8, "FR_thigh_joint": 45.8, "RL_thigh_joint": 57.3, "RR_thigh_joint": 57.3,
        "FL_calf_joint": -85.9, "FR_calf_joint": -85.9, "RL_calf_joint": -85.9, "RR_calf_joint": -85.9}

# --- 3. Everything else inside try / finally (close the app even after an error) ----------
try:
    # --- 4. Imports that need the running app ----------------------------------------------
    import math

    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    lines = []                                     # the report, built as we go

    def log(msg):
        print(msg)
        lines.append(msg)

    # --- 5. Open the barn ---------------------------------------------------------------
    ok, stage = stage_utils.open_stage(BARN)       # verified: returns (bool, Usd.Stage | None)
    if not ok or stage is None:
        raise SystemExit(f"could not open {BARN}")
    log(f"opened {BARN}")

    # --- 6. Check the physics scene: exactly ONE, and its step rate -----------------------
    # Check by TYPE, not by path: whatever it's called, there must be exactly one.
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise SystemExit(f"expected 1 PhysicsScene, found {len(scenes)}: {[str(p.GetPath()) for p in scenes]}")
    rate = scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get()
    log(f"physics scene {scenes[0].GetPath()}: {rate} steps/s")

    # --- 7. Add the Go2 (Lesson 3 pattern) -------------------------------------------------
    if not stage.GetPrimAtPath("/World"):
        raise SystemExit("/World not found: is this the barn?")
    go2 = UsdGeom.Xform.Define(stage, "/World/Go2")
    UsdGeom.XformCommonAPI(go2).SetTranslate(Gf.Vec3d(*SPAWN))
    go2.GetPrim().GetReferences().AddReference(GO2)
    # Physics lives in the "Physics" variant set (each variant a payload): select + load
    # explicitly, so the joints exist now (no GUI defaults in a headless run).
    go2.GetPrim().GetVariantSet("Physics").SetVariantSelection("physx")
    stage.Load(go2.GetPath())
    log(f"Go2 at {SPAWN}, Physics variant: {go2.GetPrim().GetVariantSet('Physics').GetVariantSelection()}")

    # --- 8. Standing pose: drive targets + initial joint states (degrees, USD units) -------
    joints = {}                                    # name -> prim, found once, reused below
    for prim in stage.Traverse():
        if prim.GetPath().pathString.startswith("/World/Go2") and prim.GetName() in POSE:
            deg = POSE[prim.GetName()]
            prim.GetAttribute("drive:angular:physics:targetPosition").Set(deg)   # where the motor pulls
            prim.GetAttribute("state:angular:physics:position").Set(deg)         # where the joint starts
            prim.GetAttribute("state:angular:physics:velocity").Set(0.0)         # start at rest
            joints[prim.GetName()] = prim
    missing = sorted(set(POSE) - set(joints))
    if missing:
        raise SystemExit(f"only {len(joints)}/12 leg joints found; missing: {missing}")
    fl = joints["FL_thigh_joint"]
    log(f"{len(joints)}/12 leg joints set; FL_thigh stiffness "
        f"{fl.GetAttribute('drive:angular:physics:stiffness').Get():.3f}/deg (expect ~1.0)")

    # Where is the articulation root? Find it instead of guessing (Lesson 3: on 'base').
    roots = [p for p in Usd.PrimRange(go2.GetPrim()) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise SystemExit(f"expected 1 articulation root under /World/Go2, found {len(roots)}")
    root_path = str(roots[0].GetPath())
    log(f"articulation root: {root_path}")

    # --- 9. Physics clock and start ---------------------------------------------------------
    log(f"SimulationManager physics dt: {SimulationManager.get_physics_dt():.5f} s (barn layer says 1/{rate})")
    app_utils.play()                               # verified; no SimulationManager.play()
    simulation_app.update()                        # let the app process Play (advances one app frame)

    # --- 10. Wrapper to READ the robot while simulating -------------------------------------
    # Created AFTER play: the articulation exists in the physics engine only once it runs.
    robot = Articulation(root_path)
    names = list(robot.dof_names)                  # the articulation's joint order
    log(f"{robot.num_dofs} DOFs: {names}")
    target_deg = [POSE[n] for n in names]          # targets in the SAME order as the readings

    # --- 11. The loop: step, measure, log ---------------------------------------------------
    dt = SimulationManager.get_physics_dt()
    steps = int(round(DURATION_S / dt))
    log_every = max(1, int(round(LOG_EVERY_S / dt)))
    t0 = SimulationManager.get_simulation_time()   # pre-loop steps happened; report relative to t0
    log(f"loop: {steps} steps of {dt:.5f} s, start sim time {t0:.3f} s")
    log("   t_s   base_z_m  max|err|_deg  worst_joint")
    for i in range(1, steps + 1):
        SimulationManager.step()                   # one physics step; no app update in the loop
        if i % log_every == 0:
            pos, _ = robot.get_world_poses()       # root link (base) pose
            z = float(pos.numpy()[0][2])
            q = robot.get_dof_positions().numpy()[0]           # RADIANS from the physics engine
            err = [math.degrees(a) - t for a, t in zip(q, target_deg)]
            k = max(range(len(err)), key=lambda j: abs(err[j]))
            t = SimulationManager.get_simulation_time() - t0
            log(f"{t:6.2f}  {z:8.4f}  {abs(err[k]):11.2f}   {names[k]} ({err[k]:+.2f})")

    # --- 12. Write the report, save the scene -----------------------------------------------
    with open(REPORT, "w") as f:                   # the result: write it first
        f.write("\n".join(lines) + "\n")
    print("report written to", REPORT)
    app_utils.stop()                               # save the authored start state, not mid-run
    simulation_app.update()
    saved = stage_utils.save_stage(SAVED_SCENE)    # verified: copies root layer to a NEW file
    print("scene saved:", saved, SAVED_SCENE)      # and fixes relative paths; barn_sim.usda untouched

# --- 13. finally: close the app ---------------------------------------------------------
finally:
    simulation_app.close()
