# Unit 2, Lesson 1, Part C: four-wheel simple robot (spec for the Python script)

Taken from the GUI-built `simple_robot_four_wheel.usda` (Isaac Sim 6.1) after the
joint limits were cleared. The robot drives forward (+X) on its rear wheels.

Script target: build the same stage with `pxr` only, using shape prims
(`UsdGeom.Cube` for the body, `UsdGeom.Cylinder` for the wheels) instead of the
GUI's meshes. Same transforms and scales, so the robot behaves the same.

## Stage

| Setting | Value |
|---|---|
| upAxis | Z |
| metersPerUnit | 1 |
| defaultPrim | `World` |

## Prims and transforms

Transforms are local (relative to the parent). xformOp order: translate, orient, scale.

| Path | GUI type | Script type | Translate | Orient (quat w, x, y, z) | Scale | Geometry |
|---|---|---|---|---|---|---|
| `/World` | Xform | Xform | – | – | – | – |
| `/World/PhysicsScene` | PhysicsScene | PhysicsScene | – | – | – | – |
| `/World/GroundPlane` | Xform | Xform | (0, 0, 0) | (1, 0, 0, 0) | (1, 1, 1) | – |
| `/World/GroundPlane/CollisionMesh` | Mesh | Mesh | – | – | – | 1 quad, points (±25, ±25, 0), grey (visual only) |
| `/World/GroundPlane/CollisionPlane` | Plane | Plane | – | – | – | axis Z, purpose guide |
| `/World/SimpleRobot` | Xform | Xform | (0, 0, **0.375**) | (1, 0, 0, 0) | (1, 1, 1) | – |
| `/World/SimpleRobot/Cube` | Mesh | **Cube** | (0, 0, 0) | (1, 0, 0, 0) | (**2, 1, 0.5**) | size 1 (extent ±0.5) |
| `/World/SimpleRobot/Front_Left_Wheel` | Mesh | **Cylinder** | (**0.5, 0.75**, 0) | (0.7071, 0.7071, 0, 0) = **+90° about X** | (**0.75, 0.75, 0.25**) | radius 0.5, height 1, axis Z |
| `/World/SimpleRobot/Front_Right_Wheel` | Mesh | Cylinder | (0.5, **−0.75**, 0) | same | same | same |
| `/World/SimpleRobot/Back_Left_Wheel` | Mesh | Cylinder | (**−0.5**, 0.75, 0) | same | same | same |
| `/World/SimpleRobot/Back_Right_Wheel` | Mesh | Cylinder | (−0.5, −0.75, 0) | same | same | same |
| `/Environment` | Xform | Xform | (0, 0, 0) | (1, 0, 0, 0) | (1, 1, 1) | – |
| `/Environment/defaultLight` | DistantLight | DistantLight | (0, 0, 0) | (0.6533, 0.2706, 0.2706, 0.6533) | (1, 1, 1) | intensity 3000, exposure 1.5, angle 1 |

Resulting sizes:
- **Body:** 2 × 1 × 0.5 m, with its bottom at z = 0.125.
- **Wheels:** radius 0.375 m, width 0.25 m. The centres are at z = 0.375, so the wheels touch the ground at the start.

## Physics APIs

| Prim | APIs | Attributes |
|---|---|---|
| PhysicsScene | (GUI/Kit adds NewtonSceneAPI, PhysxSceneAPI) | nothing authored, so the default gravity applies (−Z, 9.81) |
| CollisionPlane | PhysicsCollisionAPI | static collider: no RigidBodyAPI |
| Cube, 4 wheels | PhysicsRigidBodyAPI, PhysicsCollisionAPI | (GUI meshes also had PhysicsMeshCollisionAPI with `approximation = "convexHull"`; shape prims don't need it) |

## Joints

Each joint is a `PhysicsRevoluteJoint`. In the GUI file each one is a child of its wheel.

| Joint path | body0 | body1 | axis | localPos0 | localPos1 | localRot0 | localRot1 | Limits |
|---|---|---|---|---|---|---|---|---|
| `…/Front_Left_Wheel/Front_Left_Joint` | `/World/SimpleRobot/Cube` | `…/Front_Left_Wheel` | Y | (0.25, 0.75, 0) | (0, 0, 0) | (1, 0, 0, 0) | (0.7071, −0.7071, 0, 0) | none |
| `…/Front_Right_Wheel/Front_Right_Joint` | Cube | `…/Front_Right_Wheel` | Y | (0.25, −0.75, 0) | (0, 0, 0) | (1, 0, 0, 0) | (0.7071, −0.7071, 0, 0) | none |
| `…/Back_Left_Wheel/Back_Left_Joint` | Cube | `…/Back_Left_Wheel` | Y | (−0.25, 0.75, 0) | (0, 0, 0) | (1, 0, 0, 0) | (0.7071, −0.7071, 0, 0) | none |
| `…/Back_Right_Wheel/Back_Right_Joint` | Cube | `…/Back_Right_Wheel` | Y | (−0.25, −0.75, 0) | (0, 0, 0) | (1, 0, 0, 0) | (0.7071, −0.7071, 0, 0) | none |

How to read the joint values:
- **localPos0** is in the Cube's own *scaled* frame. With the scale (2, 1, 0.5), (0.25, 0.75, 0) is at (0.5, 0.75, 0), the wheel centre.
- **localRot1** is −90° about X. It cancels the wheel's +90° orient, so the joint frame lines up with the Cube and the joint spins about the Cube's Y.
- **Limits: none.** Don't author `lowerLimit`/`upperLimit`; the fallback is −inf/+inf, which means free spin. The GUI joint came with ±90°, so each wheel turned only a quarter turn and the robot "moved a bit and stopped".

## Drives

Rear joints only. The front wheels roll freely.

| Joint | API | damping | stiffness | targetVelocity | maxForce | type |
|---|---|---|---|---|---|---|
| Back_Left_Joint | PhysicsDriveAPI:angular | 10000 | 0 (default) | 50 °/s | inf (default) | force (default) |
| Back_Right_Joint | PhysicsDriveAPI:angular | 10000 | 0 (default) | 50 °/s | inf (default) | force (default) |

Stiffness 0 with high damping makes this a velocity drive.

## pxr classes you'll need

| Task | Class / call |
|---|---|
| Shapes | `UsdGeom.Xform`, `UsdGeom.Cube`, `UsdGeom.Cylinder`, `UsdGeom.Mesh`, `UsdGeom.Plane` |
| Translate / orient / scale | `UsdGeom.Xformable(prim).AddTranslateOp()`, `.AddOrientOp()`, `.AddScaleOp()` (orient takes a `Gf.Quatf`) |
| Light | `UsdLux.DistantLight` |
| Scene, bodies, colliders | `UsdPhysics.Scene`, `UsdPhysics.RigidBodyAPI.Apply`, `UsdPhysics.CollisionAPI.Apply` |
| Joint | `UsdPhysics.RevoluteJoint.Define` → `CreateAxisAttr`, `CreateBody0Rel().SetTargets`, `CreateLocalPos0Attr`, `CreateLocalRot1Attr` … |
| Drive | `UsdPhysics.DriveAPI.Apply(joint_prim, "angular")` → `CreateDampingAttr`, `CreateStiffnessAttr`, `CreateTargetVelocityAttr` |
| Next step (course) | `UsdPhysics.ArticulationRootAPI.Apply` on `/World/SimpleRobot` |

## Checkpoints

- [ ] The script runs from the Script Editor while the timeline is stopped (reuse the Part B guard).
- [ ] On Play, the robot settles on four wheels without a jump.
- [ ] It drives steadily in +X and doesn't stop.
- [ ] The saved `.usda` matches this spec. Diff it against the GUI file: only the geometry type and the Physx\* extras should differ.
