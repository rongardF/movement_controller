# Roadmap: movement_controller

**Milestone:** v1.0 — UR10 trajectory execution, scene management, constraints, validated in sim + hardware
**Created:** 2026-05-26
**Granularity:** Fine (8 phases)

---

## Phase 1 — Package Scaffold & Interface Definitions

**Goal:** Establish the buildable ROS2 package skeleton with all interface files and directory layout in place. Nothing executable yet — just the foundation every subsequent phase builds on.

**Covers:** PKG-01, PKG-02, PKG-03, PKG-04, PKG-05, PKG-06, UR-01, UR-02

**Plans:** 4 plans

Plans:
- [x] 01-01-PLAN.md — Hybrid package build system (CMakeLists.txt + package.xml + setup.py + setup.cfg with SKIP_INSTALL workaround)
- [x] 01-02-PLAN.md — ROS2 interface files (action/ExecuteTrajectory.action, msg/TrajectoryPath.msg, 5 srv files)
- [x] 01-03-PLAN.md — Python module skeleton (movement_controller/ package + 4 sub-package stubs with BSD-3-Clause headers)
- [x] 01-04-PLAN.md — License headers, .gitignore, smoke test (test_imports.py), CI baseline verification

**Success criteria:**
- `colcon build --symlink-install` succeeds with zero errors
- `colcon test` discovers and passes the smoke test
- All interface files compile and are importable from Python
- All source files have BSD-3-Clause headers

---

## Phase 2 — LifecycleNode & Action Server Skeleton

**Goal:** Running ROS2 node with action server wired up — accepts goals, returns results, logs lifecycle transitions. No actual planning yet.

**Covers:** ACT-01, ACT-02, ACT-03, ACT-04, ACT-05

**Plans:** 4 plans

Plans:
- [ ] 02-01-PLAN.md — LifecycleNode base (`URMovementController(LifecycleNode)` with lifecycle callbacks + parameter declarations)
- [ ] 02-03-PLAN.md — Data models (enums, `TrajectoryPathDTO`, `TrajectoryGoalDTO`, `TrajectoryGrouper`)
- [ ] 02-02-PLAN.md — Action server (ActionServer wired in `on_configure`, `_goal_callback`, `_execute_callback`, `setup.py` entry point)
- [ ] 02-04-PLAN.md — Unit tests (enums/DTOs, grouper algorithm, controller callbacks) + CMakeLists.txt test registration

**Success criteria:**
- Node starts, activates, and accepts a trajectory goal without crashing
- Concurrent goal is rejected with an error result
- Unit tests pass without hardware

---

## Phase 3 — MoveIt2 + PILZ Single-Path Execution

**Goal:** Execute a single trajectory path using MoveIt2 with PILZ planner. The look-ahead pipeline is not yet wired — this phase proves PILZ planning + execution works end-to-end in simulation.

**Covers:** MOT-01, MOT-02, MOT-05

**Plans:** 4 plans

Plans:
- [ ] 03-01-PLAN.md — MoveIt2 integration (ros-jazzy-moveit Dockerfile; MoveItPy init with timeout in on_configure; on_cleanup teardown)
- [ ] 03-02-PLAN.md — PILZ planner service (PlanResultDTO; PilzPlannerService with plan() mapping MotionTypeEnum to PILZ pipelines; unit tests)
- [ ] 03-03-PLAN.md — Single-path execution (CIRC validation in goal_callback; flatten-groups loop with plan+execute+feedback per path in execute_callback)
- [ ] 03-04-PLAN.md — Simulation smoke test (8 integration tests with mocked MoveItPy; 1-path success, plan failure, execution failure, CIRC/concurrent rejection)

**Success criteria:**
- Single LIN, PTP, and CIRC path each plan and execute without error in simulation
- Feedback sequence (`executing` → `completed`) delivered for each path
- PILZ pipeline ID is driven by the `motion_type` field in the goal

---

## Phase 4 — Look-Ahead Planning & Blended Multi-Path Execution

**Goal:** Execute multi-path trajectories with blending via `MoveGroupSequence` and zero inter-path stop time through look-ahead parallel planning.

**Covers:** MOT-02, MOT-03, MOT-04

**Plans:** 4 plans

Plans:
- [x] 04-01-PLAN.md — Foundation: PlanResultDTO extension (path_ids, blended, trajectories); PilzPlannerService constructor + node param + GetMotionSequence client; URMovementController on_configure update
- [x] 04-02-PLAN.md — Look-ahead thread: plan_all, _planning_loop, _plan_group_sequence (MotionSequenceRequest), iterate_planned_trajectories generator, cancel (D-03 through D-09)
- [x] 04-03-PLAN.md — Controller wiring: _execute_callback generator loop with group-level feedback and TEM execution; _cancel_callback; ActionServer cancel_callback wiring (D-01, D-02, D-10)
- [x] 04-04-PLAN.md — Tests: unit tests for plan_all/iterate/cancel (mock service client); updated integration tests for Phase 4 API; 3-path blended scenario; cancel scenario

**Success criteria:**
- 3-path blended trajectory executes with no stop between segments in simulation
- Look-ahead planning thread plans N+1 while N executes (verifiable via log or test hook)
- Cancellation mid-trajectory stops cleanly and returns error result

---

## Phase 5 — Motion Constraints

**Goal:** Persistent motion constraints loaded from node parameters, applied to every planning request.

**Covers:** CON-01, CON-02, CON-03, CON-04

### Plans

1. **Parameter declarations** — Declare all constraint parameters with `ParameterDescriptor` descriptions; read on `on_configure`; validate with Pydantic `ConstraintConfigDTO`
2. **Workspace bounding-box constraint** — Convert bounding-box parameters to MoveIt2 `PositionConstraint` applied to `tool0`; attach to all planning requests
3. **Joint constraints** — Convert per-joint parameter values to MoveIt2 `JointConstraint` objects; attach to all planning requests
4. **Orientation constraint** — Convert orientation tolerance parameters to MoveIt2 `OrientationConstraint` for the end-effector; attach to all planning requests
5. **Constraint tests** — Unit test that each constraint type is correctly built from parameters; integration test that planning fails when a goal violates the workspace bounding box

**Success criteria:**
- Node fails to plan a goal that violates any active constraint
- All constraint parameters have descriptors and default values
- Constraints are applied on `on_configure`; no per-move overrides possible

---

## Phase 6 — Scene Management Service

**Goal:** Callers can add/remove collision objects (primitives, meshes, attached objects) via the `ManageScene` service.

**Covers:** SCN-01, SCN-02, SCN-03, SCN-04, SCN-05, SCN-06

### Plans

1. **SceneRepository service** — Implement `SceneRepository` in `services/`; wrap MoveIt2 `PlanningSceneInterface`; provide `add_primitive`, `add_mesh`, `attach_object`, `detach_object`, `remove_object` methods
2. **ManageScene service server** — Wire `ManageScene` ROS2 service server to `SceneRepository`; map request fields to repository calls; return `{success, error_message}`
3. **Mesh loading** — Validate mesh file path exists and is readable before loading; return clear error if file not found
4. **Scene management tests** — Unit test each repository method; integration test add → attach → detach → remove lifecycle in simulation

**Success criteria:**
- Primitive and mesh objects appear in MoveIt2 planning scene after add call
- Attached object moves with robot EEF; detach returns it to world frame
- Remove call clears the object from the scene
- Invalid file path returns error result (not exception)

---

## Phase 7 — Launch Files & Simulation Validation

**Goal:** Complete launch infrastructure for both simulation and real hardware modes; full end-to-end integration test in Gazebo Harmonic.

**Covers:** UR-03, UR-04, TST-03, TST-04

### Plans

1. **Simulation launch file** — `ur10_sim.launch.py`: starts Gazebo Harmonic, spawns UR10 with `fake_hardware_interface`, launches MoveGroup with PILZ plugin enabled, launches `movement_controller` node with `use_sim_time:=true`
2. **Real hardware launch file** — `ur10_real.launch.py`: starts `ur_robot_driver` with robot IP parameter, launches MoveGroup, launches `movement_controller` node
3. **Simulation integration test** — Automated test: launch sim stack, send 2-path blended trajectory, assert feedback sequence and success result; send ManageScene add + remove, assert success
4. **Constraint validation in sim** — Integration test: configure workspace bounding box; send goal outside box; assert planning failure and error result

**Success criteria:**
- `ros2 launch movement_controller ur10_sim.launch.py` starts without errors
- `ros2 launch movement_controller ur10_real.launch.py robot_ip:=<ip>` starts without errors
- Simulation integration tests pass with `colcon test`
- All v1 requirements marked as covered

---

## Phase 8 — Real Hardware Validation

**Goal:** Validate all v1 features on a real UR10. Fix any sim-to-real discrepancies. Tag v1.0.

**Covers:** TST-05

### Plans

1. **Hardware bring-up checklist** — Document step-by-step procedure for connecting to UR10, verifying driver connection, and confirming MoveIt2 can plan
2. **LIN + PTP trajectory on hardware** — Execute a safe, low-speed 2-path blended trajectory (LIN + PTP) on real UR10; confirm feedback matches expected sequence
3. **Scene management on hardware** — Add a primitive collision object, verify robot avoids it during planning, remove it
4. **Constraint validation on hardware** — Configure workspace bounding box matching physical cell; verify planning rejects out-of-bounds goals
5. **Bug fixes & v1.0 tag** — Address any sim-to-real issues discovered; update REQUIREMENTS.md to mark TST-05 validated; create git tag `v1.0`

**Success criteria:**
- All v1 requirements with checkbox `[ ]` are marked `[x]` in REQUIREMENTS.md
- No safety incidents during hardware validation
- `v1.0` git tag exists on a clean commit

---

## Milestone Summary

| Phase | Goal | Key Requirements |
|-------|------|-----------------|
| 1 | Package scaffold & interfaces | PKG-01–06, UR-01–02 |
| 2 | LifecycleNode & action server | ACT-01–05 |
| 3 | PILZ single-path execution | MOT-01, MOT-02, MOT-05 |
| 4 | Look-ahead & blended multi-path | MOT-02–04 |
| 5 | Motion constraints | CON-01–04 |
| 6 | Scene management | SCN-01–06 |
| 7 | Launch files & sim validation | UR-03–04, TST-03–04 |
| 8 | Real hardware validation | TST-05 |

---
*Roadmap created: 2026-05-26*
*Last updated: 2026-05-26 after initialization*
