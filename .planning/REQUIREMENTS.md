# Requirements: movement_controller

**Defined:** 2026-05-26
**Core Value:** A single reliable ROS2 action that executes collision-aware, blended multi-path trajectories on a UR10 using the PILZ motion planner — working identically in Gazebo simulation and on real hardware.

## v1 Requirements

### Package Foundation

- [ ] **PKG-01**: ROS2 package exists with correct `ament_cmake_python` hybrid layout (C++ for `rosidl` interface generation, Python for node implementation)
- [ ] **PKG-02**: `package.xml` declares all runtime and test dependencies (`moveit_py`, `ur_robot_driver`, `ur_moveit_config`, `rclpy`, `pydantic`, `ament_pytest`, etc.)
- [ ] **PKG-03**: `CMakeLists.txt` generates ROS2 interfaces (`ExecuteTrajectory.action`, `ManageScene.srv`, supporting `.msg` types) via `rosidl_generate_interfaces`
- [ ] **PKG-04**: Python module layout follows project conventions (`movement_controller/`, `models/`, `enums/`, `utils/`, `services/`)
- [ ] **PKG-05**: All source files carry BSD-3-Clause license header
- [ ] **PKG-06**: Package builds with `colcon build --symlink-install` without errors in the devcontainer

### ROS2 Action Interface

- [ ] **ACT-01**: `ExecuteTrajectory` action defined in `action/ExecuteTrajectory.action` with goal containing a list of trajectory paths (`TrajectoryPath` from `msg/TrajectoryPath.msg`)
- [ ] **ACT-02**: Each `TrajectoryPath` in the goal specifies: a UUID4 path ID (string), motion type (LIN / PTP / CIRC), one target poses (`geometry_msgs/PoseStamped`), blend radius (float), cartesian speed (float, m/s) of end-effector, acceleration/de-acceleration (float, m/s2) of end-effector and circular movement related values like circular movement type (string, `interim` / `center`) and circular movement point (`geometry_msgs/Point`)
- [ ] **ACT-03**: Action feedback publishes `{status: executing|completed, trajectory_path_id: list[string]}` after each path segment begins and completes. The blended trajectory paths will be executed in one continuous motion so feedback will be sent only once when execution starts and once when execution completes. The feedback `trajectory_path_id` is a list that will contain all ID values of trajectory paths included in the blended motion.
- [ ] **ACT-04**: Action result returns `{success: bool, error_message: string, trajectory_paths_completed: list[string]}`
- [ ] **ACT-05**: Trajectory execution node is a `rclpy.lifecycle.LifecycleNode` with correct lifecycle transitions
- [ ] **ACT-06**: `ExecuteTrajectory` action goal can contain one-to-many trajectory paths and they are executed in the order that they are defined
- [ ] **ACT-07**: If `TrajectoryPath` has a non-zero/non-negative (>=0) blend radius then it must be blended together with previous `TrajectoryPath` in the goal list of trajectory paths. Exception to this is the first `TrajectoryPath` in the list – this one cannot be blended together with any previous `TrajectoryPath` so blend radius for that is ignored.

### Motion Planning & Execution

- [ ] **MOT-01**: PILZ Industrial Motion Planner plugin is used for planning (`LIN`, `PTP`, `CIRC` pipeline IDs)
- [ ] **MOT-02**: Multi-path trajectories with blending are executed via MoveIt2 `MoveGroupSequence` action
- [ ] **MOT-03**: Look-ahead planning: path >N are planned sequentially (in order) on a background thread while path N is executing
- [ ] **MOT-04**: Planned trajectories are queued; the next path executes immediately when current path completes (no re-plan latency)
- [ ] **MOT-05**: Action server rejects new goals while a trajectory is in execution; returns error result

### Motion Constraints

- [ ] **CON-01**: Node reads workspace bounding-box constraint from parameters (`workspace.x_min`, `workspace.x_max`, `workspace.y_min`, `workspace.y_max`, `workspace.z_min`, `workspace.z_max`) and applies it to all planning requests
- [ ] **CON-02**: Node reads per-joint angle constraints from parameters and applies them to all planning requests
- [ ] **CON-03**: Node reads end-effector orientation constraint from parameters (tolerated roll/pitch/yaw deviation) and applies it to all planning requests
- [ ] **CON-04**: Constraints are applied persistently for the lifetime of the node; they are not overridable per action goal
- [ ] **CON-05**: Node reads end-effector maximum cartesian speed constraint and rejects the goal if requested cartesian speed exceeds it
- [ ] **CON-06**: Node reads end-effector maximum joint speeds constraint and applies them to all planning requests

### Scene Management

- [ ] **SCN-01**: `ManageScene` service (or action) defined in `srv/` allowing callers to add or remove collision objects
- [ ] **SCN-02**: Primitive shapes (box, sphere, cylinder) can be added as collision objects with pose and size
- [ ] **SCN-03**: Mesh files (STL/DAE) can be added as collision objects given a file path and pose
- [ ] **SCN-04**: Collision objects can be attached to a robot link (e.g., grasped object on `tool0`) via the same interface
- [ ] **SCN-05**: Attached objects can be detached and returned to the world frame
- [ ] **SCN-06**: Collision objects can be removed by ID
- [ ] **SCN-07**: It is possible to modify collision-allowed matrix for all the scene objects

### UR10 Integration

- [ ] **UR-01**: Package depends on `ur_robot_driver` and `ur_moveit_config` for UR10 kinematics, URDF, and SRDF
- [ ] **UR-02**: MoveIt2 planning group name matches the `ur_moveit_config` SRDF group (`ur_manipulator`)
- [ ] **UR-03**: Launch file for **simulation** mode starts Gazebo Harmonic, `ur_robot_driver` with `fake_hardware_interface`, MoveGroup, and the `movement_controller` node with `use_sim_time:=true`
- [ ] **UR-04**: Launch file for **real hardware** mode starts `ur_robot_driver`, MoveGroup, and the `movement_controller` node targeting the real UR10 IP

### Testing & Validation

- [ ] **TST-01**: Unit tests cover Pydantic data models and utility functions using `pytest` + `ament_pytest`
- [ ] **TST-02**: Unit tests mock `moveit_py` and ROS2 interfaces; no hardware required to run unit tests
- [ ] **TST-03**: Integration test verifies `ExecuteTrajectory` action completes a 2-path blended trajectory in Gazebo simulation
- [ ] **TST-04**: Integration test verifies `ManageScene` service can add and remove a collision object in simulation
- [ ] **TST-05**: Acceptance: full trajectory execution validated on real UR10 hardware with at least one LIN and one PTP path

## v2 Requirements

### Multi-Vendor Support

- **VEND-01**: `BaseMovementController` abstract class formalised; UR10 implementation refactored to use it
- **VEND-02**: Second vendor (e.g., Fanuc) implemented against the abstract interface

### Extended Interface

- **EXT-01**: UR10e (e-Series) supported in addition to UR10 classic
- **EXT-02**: Per-move constraint overrides allowed in action goal (supplement persistent constraints)
- **EXT-03**: Named target support in trajectory paths (SRDF named configurations alongside pose waypoints)

### Operational Features

- **OPS-01**: Scene state persistence — save and restore named scene configurations
- **OPS-02**: Diagnostics topic exposing constraint state, current planning group, and connection status

## Out of Scope

| Feature | Reason |
|---------|--------|
| Gripper / end-effector actuation | Separate concern; not part of arm motion planning |
| Force/torque compliant control | Requires different control paradigm; deferred |
| REST / HTTP API | Consumers are ROS2 nodes; application-layer bridging is caller's responsibility |
| UR10e / e-Series in v1 | UR10 classic is the target; e-Series deferred |
| OMPL motion planner | Stochastic; PILZ gives deterministic industrial motion profiles |
| Per-move constraint overrides | Constraints are cell-wide configuration; per-move overrides deferred to v2 |
| MoveIt Commander | ROS 1 only — explicitly forbidden |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| PKG-01 – PKG-06 | Phase 1 | Pending |
| ACT-01 – ACT-07 | Phase 2 | Pending |
| MOT-01 – MOT-03 | Phase 3 | Pending |
| MOT-04 – MOT-05 | Phase 3 | Pending |
| CON-01 – CON-06 | Phase 4 | Pending |
| SCN-01 – SCN-07 | Phase 5 | Pending |
| UR-01 – UR-02 | Phase 1 | Pending |
| UR-03 – UR-04 | Phase 6 | Pending |
| TST-01 – TST-02 | Phases 1–5 (alongside) | Pending |
| TST-03 – TST-04 | Phase 7 | Pending |
| TST-05 | Phase 8 | Pending |

**Coverage:**
- v1 requirements: 34 total
- Mapped to phases: 34
- Unmapped: 0

---
*Requirements defined: 2026-05-26*
*Last updated: 2026-05-26 after initialization*
