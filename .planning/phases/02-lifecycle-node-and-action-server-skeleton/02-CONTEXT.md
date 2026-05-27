# Phase 2: LifecycleNode & Action Server Skeleton — Context

**Gathered:** 2026-05-27
**Status:** Ready for planning

<domain>
## Phase Boundary

Implement a running ROS2 node with full lifecycle transitions and a wired-up `ExecuteTrajectory` action server that accepts goals, sends stub feedback per execution group, and returns results. No actual MoveIt2 planning — this phase proves the lifecycle, action server, data models, and grouping logic all work before motion planning is added in Phase 3.

**In scope:** `URMovementController(rclpy.lifecycle.LifecycleNode)` with `on_configure`/`on_activate`/`on_deactivate`/`on_cleanup`; `ExecuteTrajectory` action server (async+callback); Pydantic v2 DTOs (`TrajectoryPathDTO`, `TrajectoryGoalDTO`, `FeedbackStatusEnum`, `MotionTypeEnum`); `TrajectoryGrouper` utility; lifecycle-aware goal rejection; stub feedback sequence; unit tests.

**Out of scope:** Any actual MoveIt2 planning or execution (Phase 3+), scene management (Phase 6), motion constraints (Phase 5), launch files (Phase 7). No `moveit_py` imports in this phase.

</domain>

<decisions>
## Implementation Decisions

### Parameter Declarations
- **D-01:** `on_configure` declares **only two parameters** in Phase 2:
  - `action_server_name` (string, default: `'movement_controller/execute_trajectory'`) — the action server name registered with ROS2.
  - `moveit_group_name` (string, default: `'ur_manipulator'`) — declared now as it is a universal/stable parameter needed from Phase 3 onward.
  - All constraint parameters (workspace bounds, joint limits, speeds), robot IP, and MoveIt2 connection parameters are declared in their respective later phases.

### Goal Validation
- **D-02:** Validation happens in `goal_callback` — structural check, REJECT before accept. Fast failure for callers.
- **D-03:** Phase 2 validation rules (structural only):
  1. `paths` list is non-empty
  2. Each path's `motion_type` is one of the string constants `"LIN"`, `"PTP"`, `"CIRC"`
  3. Each path's `path_id` is a non-empty string
  - Speed, blend radius range, and CIRC-specific field validity are NOT validated in Phase 2 (deferred to Phase 3 when planning runs).
- **D-04:** On validation failure: log the error at ERROR level, return `GoalResponse.REJECT`.

### Stub Execution Behavior & Feedback
- **D-05:** `execute_callback` sends the **full per-group feedback sequence** even though no real planning happens. This establishes the contract downstream consumers can build against.
- **D-06:** Feedback sequence per execution group: `{status: 'executing', trajectory_path_ids: [all ids in group]}` followed by `{status: 'completed', trajectory_path_ids: [all ids in group]}`. After all groups: return success result with `trajectory_paths_completed` = all path_ids from the goal.
- **D-07:** Blend grouping algorithm — path `t[i]` is merged into the SAME execution group as `t[i-1]` **if and only if** `t[i].blend_radius > 0` AND `i > 0`. Specifically:
  - First path (`i=0`) always starts a new group (blend_radius on first path is ignored).
  - Path `t[i]` with `blend_radius <= 0` always starts a new group.
  - Path `t[i]` with `blend_radius > 0` (and `i > 0`) merges into the group currently containing `t[i-1]`.
  - **Example:** `[t0(br=0.5), t1(br=0), t2(br=0), t3(br=0.3), t4(br=0.3), t5(br=0.3), t6(br=0)]` → groups: `[t0]`, `[t1]`, `[t2, t3, t4, t5]`, `[t6]` → 8 feedback messages.
  - Single-path groups → will use `MoveGroup` in Phase 3 (individual motion planning).
  - Multi-path groups → will use `MoveGroupSequence` in Phase 4 (blended execution).
  - **Negative blend_radius:** silently treated as `0.0` (no blending, no error).

### TrajectoryGrouper Utility
- **D-08:** Implemented as a standalone class `TrajectoryGrouper` in `movement_controller/utils/trajectory_grouper.py`.
- **D-09:** Interface: `TrajectoryGrouper.group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]` — static or class method, no state.
- **D-10:** Validation inside grouper: raises `ValueError` for empty `path_id` strings, duplicate `path_id` values, or invalid `motion_type` values. Negative `blend_radius` is silently normalized to `0.0` before grouping.
- **D-11:** Called at the **top of `execute_callback`** before sending any feedback, after goal_callback has already accepted the goal.

### Concurrent Goal Rejection
- **D-12:** `goal_callback` rejects based on **two conditions** (checked in order):
  1. **Lifecycle state:** If node is NOT in `PRIMARY_STATE_ACTIVE` (from `lifecycle_msgs.msg.State`), return `GoalResponse.REJECT` regardless of `_is_executing`.
  2. **Execution flag:** If `_is_executing` is `True` (another goal is already running), return `GoalResponse.REJECT`.
- **D-13:** `_is_executing` flag set at the entry of `execute_callback`, cleared on exit (whether success, abort, or exception). Protected by a `threading.Lock`.
- **D-14:** Lifecycle state check uses `self.get_current_state().id == State.PRIMARY_STATE_ACTIVE` (not label string comparison).

### Data Models (Pydantic v2)
- **D-15:** Four Pydantic/Enum types to implement in Phase 2:
  - `MotionTypeEnum(str, Enum)` in `movement_controller/enums/` — values: `LIN = "LIN"`, `PTP = "PTP"`, `CIRC = "CIRC"`.
  - `FeedbackStatusEnum(str, Enum)` in `movement_controller/enums/` — values: `EXECUTING = "executing"`, `COMPLETED = "completed"`.
  - `TrajectoryPathDTO(BaseModel, frozen=True)` in `movement_controller/models/` — mirrors `TrajectoryPath.msg` fields with Pydantic types; `motion_type: MotionTypeEnum`; `blend_radius` coerced to `0.0` if negative via validator; `path_id` validated non-empty.
  - `TrajectoryGoalDTO(BaseModel, frozen=True)` in `movement_controller/models/` — wraps `paths: list[TrajectoryPathDTO]`; validates non-empty list.

### Unit Tests
- **D-16:** Test targets for Phase 2 (in `tests/unit/`):
  1. Lifecycle transitions: mock `goal_callback` not callable when `UNCONFIGURED`/`INACTIVE`; transitions log correctly.
  2. Goal rejection: test each rejection condition — not ACTIVE state, `_is_executing` True.
  3. DTO validation: `TrajectoryPathDTO` rejects invalid `motion_type`, empty `path_id`, and negative `blend_radius` is normalized.
  4. `TrajectoryGrouper`: test grouping algorithm with multiple scenarios (single path, all-blend, no-blend, mixed, first-path-ignored).
  5. Stub feedback sequence: mock the action server feedback publisher; assert `executing`→`completed` pairs sent in correct order per group; assert `trajectory_paths_completed` echoes all path IDs.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Project Requirements & Architecture
- `.planning/REQUIREMENTS.md` — Full v1 requirements; ACT-01–ACT-05 are the Phase 2 scope. Verify field names, feedback semantics, and result fields match exactly.
- `.planning/PROJECT.md` — Committed architecture decisions (lifecycle node, async+callback pattern, PILZ, MoveGroupSequence, look-ahead — Phase 2 lays groundwork for all of these).
- `.planning/ROADMAP.md` §Phase 2 — Plans 1–4 with success criteria.

### Phase 1 Context (interface definitions)
- `.planning/phases/01-package-scaffold-and-interface-definitions/01-CONTEXT.md` — Decisions D-01 through D-16 define the exact interface field names, CIRC semantics, tool_frame, UUID4 path_id conventions, and Python module layout constraints that Phase 2 builds on top of.

### ROS2 Node & Action Patterns
- `.github/copilot-instructions.md` §ROS2 Node Patterns — LifecycleNode pattern, async+callback action client/server rules, `wait_for_server()` timeout requirement, parameter declaration convention.
- `.github/copilot-instructions.md` §Data Models — Pydantic v2 conventions, frozen models, DTO naming, Enum inheritance from `str, Enum`.
- `.github/copilot-instructions.md` §Error Handling — Result objects at boundaries, log before returning failure.
- `.github/copilot-instructions.md` §Testing — `pytest` + `ament_pytest`, mock all hardware, test file naming.

### External API References
- `lifecycle_msgs.msg.State` — use `State.PRIMARY_STATE_ACTIVE` (not label string) for lifecycle state checks in `goal_callback`.
- `rclpy.action.ActionServer` documentation — `goal_callback` → `GoalResponse.ACCEPT/REJECT`; `execute_callback` async pattern.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets (from Phase 1)
- `movement_controller/action/ExecuteTrajectory.action` — compiled action type; `Goal.paths: TrajectoryPath[]`, `Result.{success, error_message, trajectory_paths_completed}`, `Feedback.{status, trajectory_path_ids}`.
- `movement_controller/msg/TrajectoryPath.msg` — compiled message type with `MOTION_TYPE_LIN/PTP/CIRC` and `CIRC_TYPE_INTERIM/CENTER` string constants; all fields from D-01–D-05 of Phase 1 context.
- `movement_controller/models/__init__.py` — empty stub, ready to receive `TrajectoryPathDTO`, `TrajectoryGoalDTO`.
- `movement_controller/enums/__init__.py` — empty stub, ready to receive `MotionTypeEnum`, `FeedbackStatusEnum`.
- `movement_controller/utils/__init__.py` — empty stub, ready to receive `TrajectoryGrouper`.
- `movement_controller/services/__init__.py` — empty stub, Phase 2 does not use services.
- `tests/unit/` directory — already exists, contains `test_imports.py` from Phase 1.

### Established Patterns
- BSD-3-Clause header required on all new source files — see existing files for template.
- `ament_cmake_python` hybrid layout is already in place; no CMakeLists.txt changes needed for Python-only files.
- `ParameterDescriptor(description=...)` must be used with every `declare_parameter()` call.
- Node `__init__` accepts `node_name: str = 'ur_movement_controller'` with default, passed to `super().__init__(node_name)`.

### Integration Points
- `ur_movement_controller.py` lives at the root of the `movement_controller/` Python package (Phase 2 creates it).
- All subsequent phases (3–6) import `URMovementController` and extend it — Phase 2 establishes the class interface.
- Phase 3 will add `moveit_py` imports and planning logic inside the existing lifecycle callbacks defined here.

</code_context>

<specifics>
## Specific Ideas

- The `TrajectoryGrouper` grouping algorithm was explicitly designed during this discussion and is central to feedback semantics. The algorithm is specified exactly in D-07 — do not deviate from it.
- The blend grouping example in D-07 serves as the acceptance test for `TrajectoryGrouper` — it should be implemented as a unit test directly.
- Negative `blend_radius` → silent `0.0` normalization should happen in `TrajectoryPathDTO` validator (not in the grouper), so the grouper receives already-normalized data.
- Phase 2 stub does NOT use `time.sleep()` or introduce artificial delays — it sends feedback immediately then proceeds (no simulation of planning time).
- The `moveit_group_name` parameter default of `'ur_manipulator'` matches the `ur_moveit_config` SRDF group name from Phase 1 context D-14 (UR-02 requirement).

</specifics>

<deferred>
## Deferred Ideas

- Cross-path business rules for validation (blend radius logic, cartesian_speed > 0, CIRC validity checks) — deferred to Phase 3 when planning actually runs and these constraints can be enforced meaningfully.
- Action server cancellation handling — not in Phase 2 scope; deferred to Phase 3/4 when real execution makes cancellation meaningful.
- `TrajectoryGoalDTO.get_execution_groups()` convenience method on the model — discussed but not selected; plain `TrajectoryGrouper.group()` utility chosen instead.

</deferred>

---

*Phase: 2-LifecycleNode & Action Server Skeleton*
*Context gathered: 2026-05-27*
