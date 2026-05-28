# Phase 3: MoveIt2 + PILZ Single-Path Execution — Context

**Gathered:** 2026-05-28
**Status:** Ready for planning

<domain>
## Phase Boundary

Integrate `MoveItPy` into the `URMovementController` LifecycleNode and implement
`PilzPlannerService` so that a single trajectory path (LIN, PTP, or CIRC) is
planned via the PILZ industrial motion planner and executed against the robot.
Look-ahead and blended multi-path execution are Phase 4 — this phase proves
the PILZ planning + execution pipeline works end-to-end in simulation.

**In scope:** `MoveItPy` instantiation in `on_configure`; `PilzPlannerService`
in `services/` mapping `MotionTypeEnum` to PILZ pipeline ID; wiring the planner
service into `execute_callback` for single-path groups; CIRC path support
(both interim and center modes); `moveit_connection_timeout` node parameter;
simulation smoke test.

**Out of scope:** `MoveGroupSequence` blended execution (Phase 4), look-ahead
parallel planning (Phase 4), scene management (Phase 6), motion constraints
(Phase 5), launch files (Phase 7). No multi-path blending in this phase —
blend groups are flattened to single-path execution.

</domain>

<decisions>
## Implementation Decisions

### MoveItPy Ownership & Lifecycle
- **D-01:** `MoveItPy` is instantiated in `on_configure` by the controller
  (`URMovementController`), NOT inside `PilzPlannerService`. The controller
  owns the `MoveItPy` instance and its lifecycle.
- **D-02:** `on_configure` creates the `MoveItPy` instance, calls
  `moveit.get_planning_component('ur_manipulator')` to obtain the planning
  component, then injects the planning component into `PilzPlannerService`.
  The service is stateless — it receives the planning component and uses it
  for planning; it does not own `MoveItPy`.
- **D-03:** The MoveItPy connection timeout is a **declared node parameter**:
  `moveit_connection_timeout` (float, default: `10.0` seconds, units: seconds).
  Declared in `__init__` (not `on_configure`) with
  `ParameterDescriptor(description=...)`. All parameters are declared in
  `__init__` so they are introspectable before any lifecycle transition and
  to avoid `ParameterAlreadyDeclaredException` on re-configure.
- **D-04:** `on_configure` first probes `move_group` readiness by creating a
  temporary `GetPlanningScene` service client and calling
  `wait_for_service(timeout_sec=moveit_connection_timeout)`. If the service
  is not available within the timeout, `on_configure` logs an ERROR
  ('move_group not available after Xs — is move_group running?') and returns
  `TransitionCallbackReturn.FAILURE`. If the service is available, the client
  is destroyed and `MoveItPy()` is called directly inside a `try/except`.
  Any init exception also logs ERROR and returns FAILURE. No daemon thread
  or result_container is used — the service check makes them unnecessary.
- **D-05:** `on_cleanup` destroys the `MoveItPy` instance and sets the
  reference to `None`. `on_configure` re-creates it. Mirrors the pattern for
  `_action_server` in Phase 2.

### PilzPlannerService Interface
- **D-06:** `PilzPlannerService` is implemented in
  `movement_controller/services/pilz_planner_service.py` as a plain Python
  class (not a ROS2 node, not a ROS2 service). It receives the planning
  component (`PlanningComponent`) via its constructor. Interface:
  ```python
  class PilzPlannerService:
      def __init__(self, planning_component: PlanningComponent) -> None: ...
      def plan(self, path_dto: TrajectoryPathDTO) -> PlanResult: ...
  ```
  `PlanResult` is an internal dataclass/model that indicates success/failure
  and carries the robot trajectory on success.
- **D-07:** `PilzPlannerService.plan()` maps `MotionTypeEnum` to PILZ
  pipeline parameters:
  - `LIN` → `pipeline_id='pilz_industrial_motion_planner'`, `planner_id='LIN'`
  - `PTP` → `pipeline_id='pilz_industrial_motion_planner'`, `planner_id='PTP'`
  - `CIRC` → `pipeline_id='pilz_industrial_motion_planner'`, `planner_id='CIRC'`
    (with additional `circ_type` / `circ_point` handling — see D-10 below)
- **D-08:** The service sets `start_state_to_current_state()` at the start of
  each `plan()` call (fresh start, current robot state as the planning start).
- **D-09:** The controller holds `self._moveit` (the `MoveItPy` instance) and
  `self._planner_service` (the `PilzPlannerService`). After planning succeeds,
  the controller calls `self._moveit.execute(trajectory, blocking=True,
  controllers=[])`.

### CIRC Path Planning
- **D-10:** Both CIRC modes are supported in Phase 3:
  - `circ_type == 'interim'` (value from `CIRC_TYPE_INTERIM` constant in
    `TrajectoryPath.msg`) → pass `circ_point` as the interim waypoint to the
    PILZ CIRC planner constraint.
  - `circ_type == 'center'` (value from `CIRC_TYPE_CENTER` constant) → pass
    `circ_point` as the circle center to the PILZ CIRC planner constraint.
- **D-11:** If a path has `motion_type == 'CIRC'` but `circ_type` is empty or
  unrecognized, the goal is **rejected in `goal_callback`** with a clear error
  message identifying the path_id and the invalid `circ_type` value. Fail-fast
  before accept — consistent with Phase 2's D-03 validation philosophy.
- **D-12:** CIRC validation is added to `goal_callback`'s structural validation
  block (alongside the Phase 2 D-03 checks). The `TrajectoryGoalDTO.from_ros_msg`
  path keeps the same interface; the controller adds explicit CIRC checks before
  calling it, OR the DTO validator is extended to cover CIRC `circ_type`
  validation. Either approach is acceptable — researcher/planner decides based
  on moveit_py CIRC API findings.

### Multi-Path Group Behavior in Phase 3
- **D-13:** `execute_callback` flattens multi-path groups to individual paths
  before iterating. The call to `TrajectoryGrouper.group()` is kept (the grouper
  output is still computed), but in Phase 3 each path is executed as a
  single-path unit regardless of group membership. The flatten logic lives in
  `execute_callback`; the grouper is NOT changed.
- **D-14:** Rationale: Phase 4 will replace the flatten-and-execute-individually
  logic with `MoveGroupSequence` using the grouper output directly. Phase 3
  intentionally produces stop-start execution for blend groups rather than
  rejecting them — callers don't need to change their goals between Phase 3 and
  Phase 4.
- **D-15:** Feedback sequence for each path (regardless of blend group
  membership): `{status: 'executing', trajectory_path_ids: [path_id]}` →
  `{status: 'completed', trajectory_path_ids: [path_id]}`. Same contract as
  Phase 2 stub, now backed by real execution.

### Planning Failure Strategy
- **D-16:** **Fail-fast on first planning failure.** If `PilzPlannerService.plan()`
  fails for any path, `execute_callback` immediately aborts the goal:
  return `success=False`, `error_message` includes the failed `path_id` and
  the planner error. No partial completion reported; the caller must resend
  the full goal.
- **D-17:** Execution errors (planning succeeds but `moveit.execute()` fails) also
  abort the goal immediately with no retry. `error_message` identifies the
  path_id in flight and describes the execution error.
- **D-18:** Both planning and execution errors are logged at ERROR level before
  the result is returned — consistent with Phase 2 error handling conventions.

### Simulation Test Approach
- **D-19:** Plan 4 (simulation smoke test) mocks `MoveItPy` at the Python level.
  The test patches `MoveItPy` and the planning component so no real `move_group`
  node is required. The test sends a 1-path goal and asserts:
  1. Both feedback messages received (`executing` → `completed`).
  2. `result.success == True`.
  3. `result.trajectory_paths_completed` echoes the path_id.
  This is consistent with the Phase 2 test pattern (no hardware required for
  unit/integration tests).

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Project Requirements & Architecture
- `.planning/REQUIREMENTS.md` — MOT-01, MOT-02, MOT-05 are the Phase 3 scope.
  Read these to verify planning pipeline names, execution semantics, and
  feedback contract.
- `.planning/PROJECT.md` — PILZ over OMPL decision, MoveGroupSequence for Phase
  4 (not Phase 3), look-ahead deferred to Phase 4. These are LOCKED decisions.
- `.planning/ROADMAP.md` §Phase 3 — Plans 1–4 with success criteria.

### Phase Context (prior phases)
- `.planning/phases/02-lifecycle-node-and-action-server-skeleton/02-CONTEXT.md`
  — D-07 (grouping algorithm), D-15 (data models), D-16 (unit test targets).
  Phase 3 builds directly on these.
- `.planning/phases/01-package-scaffold-and-interface-definitions/01-CONTEXT.md`
  — D-01 through D-05 for `TrajectoryPath.msg` field semantics (`circ_type`,
  `circ_point`, `tool_frame`, `target_pose`). Phase 3 CIRC planning depends on
  these field definitions.

### MoveIt2 & PILZ API
- `.github/copilot-instructions.md` §MoveIt2 Python API (moveit_py) — the
  `MoveItPy` instantiation pattern, `get_planning_component`, `plan_and_execute`
  context manager, and `execute()` call. **MUST NOT use MoveIt Commander.**
- `.github/copilot-instructions.md` §ROS2 Node Patterns — `wait_for_server()`
  timeout requirement, parameter declaration convention, `DependencyFailure`
  pattern.
- `.github/rules/ros2-jazzy.md` — Full `moveit_py` pattern and correct planning
  group name for UR10e. Read before any `moveit_py` API calls.
- `.github/copilot-instructions.md` §Error Handling — Result objects at
  boundaries, log before returning failure.

### Testing
- `.github/copilot-instructions.md` §Testing — `pytest` + `ament_pytest`, mock
  all hardware interfaces, `tests/unit/` and `tests/integration/` layout.
- `.github/rules/testing.md` — Full testing conventions including how to mock
  `MoveItPy` for tests without a running `move_group` node.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `movement_controller/ur_movement_controller.py` — `URMovementController`
  with `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`,
  `_goal_callback`, `_execute_callback`. Phase 3 extends `on_configure` (adds
  `MoveItPy` instantiation + 1 new parameter) and replaces the stub execution
  loop in `_execute_callback`.
- `movement_controller/models/trajectory_path_dto.py` — `TrajectoryPathDTO`
  with `motion_type: MotionTypeEnum`, `target_pose: PoseStamped`,
  `circ_type: str`, `circ_point: geometry_msgs/Point`, `tool_frame: str`,
  `blend_radius: float`. The CIRC fields are already there.
- `movement_controller/models/trajectory_goal_dto.py` — `TrajectoryGoalDTO`
  with `from_ros_msg()`. Phase 3 may extend validation here for CIRC.
- `movement_controller/enums/motion_type_enum.py` — `MotionTypeEnum` with
  `LIN`, `PTP`, `CIRC`. Maps directly to PILZ planner IDs.
- `movement_controller/utils/trajectory_grouper.py` — `TrajectoryGrouper.group()`
  already implemented and tested. Used but NOT changed in Phase 3.
- `movement_controller/services/__init__.py` — empty stub. `PilzPlannerService`
  is added here in Phase 3.

### Established Patterns
- BSD-3-Clause header required on all new source files.
- `ParameterDescriptor(description=...)` required on every `declare_parameter()`
  call.
- Error handling: `try/except`, log ERROR, return failure result object.
- `_is_active` lifecycle guard already checks before accepting goals — Phase 3
  does not change this.
- Pydantic `frozen=True` on all DTOs.

### Integration Points
- `on_configure` is the insertion point for `MoveItPy` initialization. The
  existing `_action_server` creation and parameter declarations remain; add
  `moveit_connection_timeout` parameter and `MoveItPy` instantiation after them.
- `_execute_callback`'s inner loop is the insertion point for real planning.
  Replace the stub feedback loop with: flatten groups → for each path →
  plan via `PilzPlannerService` → execute via `self._moveit.execute()` →
  send `executing` feedback before execute, `completed` after.
- `on_cleanup` gains a `self._moveit = None` assignment after
  `self._action_server.destroy()`.

</code_context>

<specifics>
## Specific Ideas

- `MoveItPy` does NOT create a `move_group` node — it connects to one that is
  already running. In simulation, this is provided by `fake_hardware_interface`
  + `ur_moveit_config`. In the Phase 3 smoke test, `MoveItPy` itself is mocked.
- `PilzPlannerService` is a plain Python class, not a ROS2 service node. The
  name reflects its domain role, not its ROS2 type.
- The PILZ pipeline is selected per-path inside `PilzPlannerService.plan()` by
  setting pipeline attributes on the planning component before calling `plan()`.
- CIRC path planning requires passing the `circ_point` (from `TrajectoryPathDTO`)
  as either an interim waypoint or circle center via PILZ constraint objects —
  researcher must consult moveit.picknik.ai for exact API before implementing.
- The `tool_frame` field in `TrajectoryPathDTO` (default empty → `tool0`) should
  be used as the `pose_link` in `set_goal_state()`. If empty, default to `tool0`.

</specifics>

<deferred>
## Deferred Ideas

- CIRC validation could be moved into `TrajectoryPathDTO` validator — acceptable
  in Phase 3 if the researcher finds it cleaner. Either location (DTO validator
  or `goal_callback`) is deferred to researcher/planner to decide.
- Action server cancellation handling — still deferred (from Phase 2); becomes
  relevant in Phase 3/4 when real execution makes cancellation meaningful.
  Not in Phase 3 scope.
- Retry on planning failure — discussed, rejected. Fail-fast only in Phase 3.
- Blended execution feedback (signaling when stop-start happens) — `blending_skipped`
  status discussed, rejected for Phase 3. Callers can't distinguish blended vs
  non-blended until Phase 4 ships.

</deferred>

---

*Phase: 3-MoveIt2-PILZ-Single-Path-Execution*
*Context gathered: 2026-05-28*
