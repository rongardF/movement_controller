# Phase 5: Motion Constraints — Context

**Gathered:** 2026-06-01
**Status:** Ready for planning

<domain>
## Phase Boundary

Persistent motion constraints loaded from node parameters on `on_configure`,
applied to every MoveIt2 planning request via `MotionSequenceItem.constraints`.

**In scope:**
- Declaring all constraint parameters with `ParameterDescriptor` descriptions in `__init__`
- Reading and validating parameters into a `ConstraintConfigDTO` on `on_configure`
- Building MoveIt2 `Constraints` objects from validated config; injecting per `MotionSequenceItem`
- Workspace bounding-box constraint → `PositionConstraint` on the path's `tool_frame` link
- Per-joint angle constraints → `JointConstraint` objects (names + lower/upper arrays)
- End-effector orientation constraint → `OrientationConstraint` on the path's `tool_frame` link
- Speed enforcement: per-path `cartesian_speed` passed to PILZ; node-level max cap validated at `_goal_callback`
- Acceleration enforcement: same pattern as cartesian speed
- Unit tests for each constraint type; integration test for workspace bounding-box violation

**Out of scope:** Scene management (Phase 6), launch files (Phase 7), real hardware (Phase 8).
No per-move constraint overrides in goal — constraints are node-level only.

</domain>

<decisions>
## Implementation Decisions

### D-01: Constraint Ownership
`URMovementController` reads parameters, builds `ConstraintConfigDTO` in `on_configure`,
and calls `self._planner_service.set_constraints(dto)`. `PilzPlannerService` owns the
MoveIt2 constraint objects and injects them into every `MotionSequenceItem` it constructs.
`URMovementController` does not touch `moveit_msgs.Constraints` directly.

### D-02: Constraint Injection Point
Constraints are injected as **path constraints** on each `MotionSequenceItem.constraints`
field inside `PilzPlannerService._build_sequence_request()` (or equivalent). Every item in
the sequence gets the full set of active constraints. No `set_path_constraints()` on the
planning component is used — the `MotionSequenceItem` fields are sufficient.

### D-03: Parameter Namespace
All constraint parameters use **dot notation namespacing**:
```
constraints.workspace.x_min     (default: -inf)
constraints.workspace.x_max     (default: +inf)
constraints.workspace.y_min     (default: -inf)
constraints.workspace.y_max     (default: +inf)
constraints.workspace.z_min     (default: -inf)
constraints.workspace.z_max     (default: +inf)
constraints.joint.names         (default: [] — empty list)
constraints.joint.lower_limits  (default: [] — empty list, radians)
constraints.joint.upper_limits  (default: [] — empty list, radians)
constraints.orientation.tolerance_x  (default: 2*pi — unconstrained)
constraints.orientation.tolerance_y  (default: 2*pi — unconstrained)
constraints.orientation.tolerance_z  (default: 2*pi — unconstrained)
constraints.max_cartesian_speed  (default: 0.0 — unconstrained)
constraints.max_acceleration     (default: 0.0 — unconstrained)
constraints.joint.max_velocities (default: [] — empty list, rad/s per joint)
```
All declared with `ParameterDescriptor(description=...)`.

**Workspace default values:** `x_min/y_min/z_min` default to `-1e9`; `x_max/y_max/z_max` default to `+1e9`.
ROS2 `float64` parameters do not support Python `float('inf')`. Use large float sentinels instead.
`ConstraintConfigDTO` treats workspace as "disabled" when the min/max span covers the full
`-1e9` to `+1e9` range (i.e., when defaults are unchanged). Researcher must define the
exact disable detection rule (e.g., `x_max - x_min >= 2e9` → skip `PositionConstraint`).

### D-04: Sentinel Defaults Signal Disabled
No separate `enabled` boolean parameters. Constraints are skipped when at their
sentinel/default values:
- Workspace: `x_min == -inf` and `x_max == +inf` → skip `PositionConstraint`
- Joint: `names == []` (empty) → skip `JointConstraint`
- Orientation: `tolerance_x/y/z == 2*pi` → skip `OrientationConstraint`
- Speed/acceleration: `max == 0.0` → skip goal-callback validation

### D-05: Orientation Constraint Reference Link
The `OrientationConstraint.link_name` is set to the **path's `tool_frame`** from
`TrajectoryPathDTO`. This means each `MotionSequenceItem` gets an orientation
constraint referencing the same link it is planned for. The constraint tolerances
come from `ConstraintConfigDTO`; the link name comes from the per-path DTO.

### D-06: Speed and Acceleration Enforcement
Two-step for each path's `cartesian_speed` and `acceleration`:
1. **PILZ execution speed:** Per-path `cartesian_speed` (m/s) is passed into the
   `MotionSequenceItem` request fields so PILZ controls execution speed for that link.
   **Researcher must verify** the correct PILZ/`GetMotionSequence` mechanism
   (candidates: `LimitMaxCartesianLinkSpeed` planning adapter, `MotionPlanRequest`
   velocity field, or `CartesianSpeedLimitedConstraint`).
2. **Node-level cap:** At `_goal_callback`, before the goal is accepted, iterate all
   paths. If any path's `cartesian_speed > constraints.max_cartesian_speed` (when max > 0),
   reject the entire goal immediately with `result.success = False` and a descriptive
   `error_message`. Same for `acceleration`. `cartesian_speed == 0.0` on a path means
   "unspecified" — skip the check for that path.

### D-07: Speed Violation Error Message Format
```
Path '{path_id}' cartesian_speed {actual} m/s exceeds node maximum {max} m/s
(constraints.max_cartesian_speed)
```
Includes: path_id, actual value with unit, node max with unit, parameter name.
Same format for acceleration violations (replace `cartesian_speed` / `m/s` with
`acceleration` / `m/s²` and `constraints.max_acceleration`).

### D-08: Joint Constraint Parameterization
Three parameters:
- `constraints.joint.names` — `string[]` in any order (e.g. `['shoulder_pan_joint', 'elbow_joint']`)
- `constraints.joint.lower_limits` — `float64[]` in matching order (radians)
- `constraints.joint.upper_limits` — `float64[]` in matching order (radians)

`ConstraintConfigDTO` validates that all three arrays have the same length. If
`constraints.joint.names` is empty, no joint constraints are applied.

### D-09: Invalid Parameter Handling
If Pydantic validation fails in `ConstraintConfigDTO` (e.g., `x_min > x_max`,
array length mismatch between `names`, `lower_limits`, `upper_limits`):
- Log the `ValidationError` details at ERROR level
- Return `TransitionCallbackReturn.FAILURE` from `on_configure`
- Node does not proceed to active state

### D-10: Lifecycle Re-configure Behavior
On each `on_configure`, constraint configuration is rebuilt **fresh from parameters**.
No caching: `ConstraintConfigDTO` is constructed anew and `set_constraints()` is
called on the planner service again. This ensures lifecycle restart picks up any
parameter changes made between deactivate and re-configure.

### D-12: Workspace Bound Sentinels
ROS2 `float64` parameters do not support Python `float('inf')`. Workspace bound parameters
default to large float sentinels: `x_min/y_min/z_min = -1e9`, `x_max/y_max/z_max = +1e9`.
`ConstraintConfigDTO` considers the workspace constraint **disabled** when the defaults are
unchanged (sentinel range ≥ 2e9 on any axis). When disabled, no `PositionConstraint` is
added to `MotionSequenceItem.constraints`. This avoids creating a technically-valid but
practically unconstrained bounding-box that adds planning overhead.

### D-13: Per-Joint Velocity Limits
A new parameter `constraints.joint.max_velocities float64[]` (default: `[]`) is added alongside
the existing joint position arrays. It shares the same `constraints.joint.names` array for
joint identification — all three arrays (`names`, `lower_limits`, `upper_limits`, `max_velocities`)
must have the same length when any is non-empty (validated by `ConstraintConfigDTO`).
- **Enforcement:** Planning-time only. Per-joint velocity limits are passed to MoveIt2 as
  hints (e.g., `JointConstraint` velocity fields or velocity scaling per joint). No goal
  rejection in `_goal_callback` for velocity limits — PILZ respects velocity limits from
  the robot model by default; this overrides on a per-joint basis.
- **Default:** `[]` (empty) means unconstrained — same sentinel pattern as joint position arrays.
- **Researcher must verify:** the correct MoveIt2/PILZ API field for per-joint velocity
  overrides in a `MotionSequenceItem` (e.g., `MotionPlanRequest` velocity scaling vs
  separate constraint type).

### D-11: ConstraintConfigDTO Location
New Pydantic model `ConstraintConfigDTO` in `movement_controller/models/constraint_config_dto.py`.
Frozen. All fields with `Field(description=...)`. Imports: `float`, `list[str]`, `list[float]`.
Validated by Pydantic: arrays same length, x_min ≤ x_max, etc.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Project Requirements & Architecture
- `.planning/REQUIREMENTS.md` — CON-01 through CON-06 are the Phase 5 scope.
  CON-04 confirms constraints are persistent, not per-move.
- `.planning/PROJECT.md` — "Constraints as node parameters" is a LOCKED decision.
  Also confirms UR10 only, BSD-3-Clause license, Pydantic v2 for all DTOs.
- `.planning/ROADMAP.md` §Phase 5 — 5 plans with success criteria.

### Phase Context (prior phases — read for patterns)
- `.planning/phases/04-look-ahead-planning-and-blended-multi-path-execution/04-CONTEXT.md`
  — D-07 (`MotionSequenceRequest` code path), D-08 (state propagation for look-ahead),
  D-03 (PilzPlannerService owns background thread). Phase 5 extends `PilzPlannerService`
  by adding `set_constraints()` method used in `_build_sequence_request()`.
- `.planning/phases/03-moveit2-pilz-single-path-execution/03-CONTEXT.md`
  — `_build_pose_goal_constraints()` pattern in `PilzPlannerService` (existing Constraints
  building example). Phase 5 adds path constraints alongside existing goal constraints.
- `.planning/phases/02-lifecycle-node-and-action-server-skeleton/02-CONTEXT.md`
  — Parameter declaration pattern (declared in `__init__`, read in `on_configure`).

### MoveIt2 & PILZ API (Researcher MUST verify before planning)
- `.github/copilot-instructions.md` §MoveIt2 Python API (moveit_py) — existing MoveItPy pattern.
  **Researcher must verify:** correct PILZ/`GetMotionSequence` mechanism for per-path
  cartesian speed (`LimitMaxCartesianLinkSpeed` adapter vs `MotionPlanRequest` fields vs
  `CartesianSpeedLimitedConstraint` in `Constraints`). This is the key unknown for D-06.
- `.github/rules/ros2-jazzy.md` — Full `moveit_py` pattern, UR10 planning group name.
- `.github/copilot-instructions.md` §ROS2 Node Patterns — parameter declaration with
  `ParameterDescriptor`, `on_configure` lifecycle pattern.
- `.github/copilot-instructions.md` §Error Handling — result objects at boundaries.
- `.github/copilot-instructions.md` §Data Models — Pydantic v2, frozen, `Field(description=...)`,
  `DTO` suffix for internal models, `Enum` for enumerations.

### Testing
- `.github/copilot-instructions.md` §Testing — `pytest` + `ament_pytest`, mock hardware,
  `tests/unit/` and `tests/integration/` layout.
- `.github/rules/testing.md` — mocking `MoveItPy`; test structure conventions.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `movement_controller/ur_movement_controller.py` — `URMovementController`:
  - Parameters declared in `__init__` (lines 74–83). Add new constraint parameters here.
  - `on_configure` (line 91+) reads params and initialises `PilzPlannerService`. After Phase 5,
    also builds `ConstraintConfigDTO` and calls `self._planner_service.set_constraints(dto)`.
  - `_goal_callback` — speed/acceleration validation added here before goal acceptance.
- `movement_controller/services/pilz_planner_service.py` — `PilzPlannerService`:
  - Already imports `Constraints`, `OrientationConstraint`, `PositionConstraint`,
    `JointConstraint` (verify JointConstraint import — may need adding).
  - `_build_pose_goal_constraints()` — existing Constraints building pattern (goal constraints).
  - Phase 5 adds: `set_constraints(dto: ConstraintConfigDTO)` method; internal
    `_build_path_constraints(tool_frame: str) -> Constraints` helper; injection in
    `_build_sequence_request()` per `MotionSequenceItem`.

### New Files for Phase 5
- `movement_controller/models/constraint_config_dto.py` — `ConstraintConfigDTO` (frozen Pydantic v2)
  Workspace box (±1e9 sentinels), joint arrays (names + lower + upper + max_velocities), orientation
  tolerances, max_cartesian_speed, max_acceleration.

### Established Patterns
- **Parameter declaration:** All parameters declared in `__init__` with `ParameterDescriptor`.
  Values read in `on_configure`. Never declare in `on_configure` (raises
  `ParameterAlreadyDeclaredException` on re-configure).
- **Fail-fast:** If `ConstraintConfigDTO` validation fails → `on_configure` returns
  `TransitionCallbackReturn.FAILURE` (same pattern as MoveItPy timeout failure in Phase 3).
- **BSD-3-Clause header:** Required on all new source files.

### Integration Points
- `URMovementController.__init__` → add ~15 new `declare_parameter()` calls in the
  `# region: parameters` block (including `constraints.joint.max_velocities`).
- `URMovementController.on_configure` → after `PilzPlannerService` init, read constraint
  params, build `ConstraintConfigDTO`, call `planner_service.set_constraints(dto)`.
- `URMovementController._goal_callback` → add per-path speed/acceleration validation loop
  before the lock check (or after goal fetch — before acknowledgment).
- `PilzPlannerService._plan_group_sequence` (or equivalent) → inject path constraints into
  each `MotionSequenceItem` from stored `ConstraintConfigDTO`.

</code_context>

<specifics>
## Specific Ideas / User Notes

- The user explicitly asked about `LimitMaxCartesianLinkSpeed` planning adapter —
  this must be researched before planning. If the adapter is needed, it may require
  changes to the MoveIt2 planning pipeline configuration (not just Python code).
- Orientation constraint's `link_name` is dynamic (per-path `tool_frame`), not fixed to
  `tool0`. This makes it more flexible and correct for multi-tool setups even within v1.
- Per-path `cartesian_speed` is a genuine execution speed input to PILZ (not just a
  validation value). Researcher must confirm which PILZ API field to set.
- Workspace bound defaults use large float sentinels (±1e9) not `float('inf')` —
  ROS2 float64 parameters do not support Python infinity.
- `constraints.joint.max_velocities` is a new per-joint velocity limit array added in
  the update discussion — shares `constraints.joint.names` for joint identification;
  all four joint arrays must have matching lengths when any is non-empty.
- Researcher must verify the correct PILZ/MoveIt2 API for per-joint velocity overrides
  in `MotionSequenceItem` (D-13 open research item).
</specifics>
