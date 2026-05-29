# Phase 4: Look-Ahead Planning & Blended Multi-Path Execution — Context

**Gathered:** 2026-05-29
**Status:** Ready for planning

<domain>
## Phase Boundary

Upgrade `URMovementController` and `PilzPlannerService` so that multi-path
trajectories execute with seamless blending via `MoveGroupSequence` and zero
inter-path stop time through look-ahead parallel planning.

**In scope:** Expanding `PilzPlannerService` to own a background planning thread
and thread-safe queue; `plan_all()` / `iterate_planned_trajectories()` /
`cancel()` API on the service; `MotionSequenceRequest` for all groups (blended
and single-path alike); state-propagation for look-ahead (chaining predicted
end-state from plan N as start-state for plan N+1); updated `_execute_callback`
in `URMovementController` to consume the generator; group-level feedback contract;
non-blocking `cancel_callback` on the action server; multi-path integration test.

**Out of scope:** Motion constraints (Phase 5), scene management (Phase 6),
launch files (Phase 7), real hardware validation (Phase 8). No per-move
constraint overrides in this phase.

</domain>

<decisions>
## Implementation Decisions

### Blend-Group Feedback Contract
- **D-01:** Feedback is **group-level**. When a blend group `[A, B, C]` begins
  executing, the controller publishes one `Feedback` message:
  `{status: 'executing', trajectory_path_ids: ['A', 'B', 'C']}`.
  When the group finishes, one more: `{status: 'completed', trajectory_path_ids:
  ['A', 'B', 'C']}`. No per-path sub-messages within a blend group — the motion
  is seamlessly blended and cannot be decomposed per segment.
- **D-02:** `result.trajectory_paths_completed` in the final result is a **flat
  list** of all completed path IDs across all groups, in execution order. On
  failure mid-trajectory, it carries the IDs from all **fully-completed** groups
  that ran before the failure (partial completion is reported, so callers know
  what succeeded). On success it contains all path IDs from the goal.

### PilzPlannerService Expansion (Look-Ahead Thread)
- **D-03:** The background planning thread and queue live **inside
  `PilzPlannerService`** — not in `URMovementController` and not in a new
  service class. `PilzPlannerService` is expanded rather than replaced.
- **D-04:** `PilzPlannerService.plan_all(groups: list[list[TrajectoryPathDTO]])`
  starts a **fresh** background thread and `queue.Queue` per goal invocation.
  A new thread + queue is created each time `plan_all()` is called; they do not
  persist between goals. The thread processes groups sequentially in order.
- **D-05:** `PilzPlannerService.iterate_planned_trajectories()` is a **generator
  method** that yields `PlanResultDTO` from the queue, blocking if nothing is
  available yet. It terminates when it dequeues a `StopIteration` sentinel
  object. The controller calls `plan_all()` once, then drives execution by
  iterating the generator — one yield per group.
- **D-06:** `PlanResultDTO` gains two new fields:
  - `path_ids: list[str]` — the path IDs this result covers (populated by
    the service from the input group). The controller uses this for feedback
    construction without tracking groups separately.
  - `blended: bool` — `True` if the group has more than one path (blend group),
    `False` for single-path groups. Allows downstream code to distinguish
    quickly without re-inspecting the group size.

### MotionSequenceRequest Usage
- **D-07:** **All groups** (including single-path groups with `blend_radius=0`,
  size 1) are planned via `MotionSequenceRequest`. Uniform code path in the
  background thread. PILZ's sequence planner supports 1-item sequences.
  `PlanResultDTO.blended` is `False` for size-1 groups but the planning API
  call is identical.

### State Propagation for Look-Ahead
- **D-08:** The first group is planned with `start_state = current robot state`
  (called at the start of the background thread, before any group is planned).
  After planning each group, the thread extracts the **final joint state** from
  the last waypoint of the planned trajectory and stores it as
  `last_predicted_state`. This becomes the `start_state` for planning the next
  group. Chain: `current → end(group0) → end(group1) → ...`. Just a single
  variable — no dictionary needed.

### Cancellation
- **D-09:** `PilzPlannerService.cancel()` method implements thread-safe
  cancellation:
  1. Sets a `threading.Event` cancellation flag.
  2. Thread-safely drains (clears) the queue.
  3. Pushes a `StopIteration` sentinel into the queue so
     `iterate_planned_trajectories()` terminates cleanly.
  Inside the background thread, after each group is planned and pushed to the
  queue, the thread checks the cancellation event — if set, the thread shuts
  down without planning further groups.
- **D-10:** The ROS2 `ActionServer` is configured with a `cancel_callback`.
  This callback triggers the cancellation sequence and **must be non-blocking**:
  it calls `self._planner_service.cancel()` (fast — sets event + drains queue +
  pushes sentinel) and returns immediately. It does **not** wait for the
  background thread to join. In-flight `MoveGroupSequence` execution is
  interrupted by MoveItPy's built-in stop mechanism — no explicit `moveit.stop()`
  call is needed.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Project Requirements & Architecture
- `.planning/REQUIREMENTS.md` — MOT-02, MOT-03, MOT-04 are the Phase 4 scope.
  Read these to verify blending, look-ahead, and queue semantics.
- `.planning/PROJECT.md` — `MoveGroupSequence` for blending and look-ahead
  parallel planning are LOCKED decisions. Also confirms abstract base design,
  UR10 only in v1.
- `.planning/ROADMAP.md` §Phase 4 — 4 plans with success criteria. Success
  criteria include: 3-path blended no-stop execution; look-ahead used
  (verifiable via log or test hook); clean cancellation.

### Phase Context (prior phases)
- `.planning/phases/03-moveit2-pilz-single-path-execution/03-CONTEXT.md` —
  D-01 through D-19 govern the existing `PilzPlannerService`, `MoveItPy`
  ownership, CIRC handling, feedback contract, and simulation test approach.
  Phase 4 EXPANDS on this — do not duplicate; reference what changes.
- `.planning/phases/02-lifecycle-node-and-action-server-skeleton/02-CONTEXT.md`
  — grouping algorithm (D-07), data model decisions (D-15), feedback contract
  origin (D-16). `TrajectoryGrouper` is reused unchanged.
- `.planning/phases/01-package-scaffold-and-interface-definitions/01-CONTEXT.md`
  — `TrajectoryPath.msg` field semantics: `blend_radius`, `path_id`,
  `motion_type`, `target_pose`, `tool_frame`, `circ_type`, `circ_point`.

### MoveIt2 & PILZ API (Researcher MUST verify these before planning)
- `.github/copilot-instructions.md` §MoveIt2 Python API (moveit_py) — existing
  `MoveItPy` pattern. Researcher must verify: (a) how `MotionSequenceRequest`
  is built in `moveit_py`; (b) whether `moveit_py` exposes
  `MoveGroupSequence` directly or requires a raw `moveit_msgs/action/
  MoveGroupSequence` action client; (c) how to set `start_state` from a
  `RobotState` object (not `set_start_state_to_current_state()`); (d) how to
  extract end-state joint positions from a planned trajectory.
- `.github/rules/ros2-jazzy.md` — Full `moveit_py` pattern, correct planning
  group name for UR10. Cancel callback pattern for ROS2 action server.
- `.github/copilot-instructions.md` §ROS2 Node Patterns — async + callback
  pattern for action clients; `cancel_callback` signature and non-blocking
  requirement.
- `.github/copilot-instructions.md` §Error Handling — result objects at
  boundaries, log before returning failure.

### Testing
- `.github/copilot-instructions.md` §Testing — `pytest` + `ament_pytest`, mock
  hardware, `tests/unit/` and `tests/integration/` layout.
- `.github/rules/testing.md` — mocking `MoveItPy` for tests without a running
  `move_group` node; how to verify look-ahead was used (test hook strategy).

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `movement_controller/ur_movement_controller.py` — `URMovementController` with
  existing `_execute_callback` (the per-path loop) and `_goal_callback`. Phase 4
  replaces the inner execution loop; lifecycle callbacks and goal validation are
  unchanged. The `cancel_callback` parameter needs to be added to `ActionServer`
  construction in `on_configure`.
- `movement_controller/services/pilz_planner_service.py` — `PilzPlannerService`
  with `plan(path_dto) → PlanResultDTO`. Phase 4 adds `plan_all()`,
  `iterate_planned_trajectories()`, and `cancel()` alongside the existing
  `plan()`. `plan()` may remain for compatibility with unit tests from Phase 3
  or can be removed if the test strategy changes — researcher/planner decides.
- `movement_controller/utils/trajectory_grouper.py` — `TrajectoryGrouper.group()`
  is unchanged. The controller still calls it to produce groups before passing
  them to `plan_all()`.
- `movement_controller/models/plan_result_dto.py` — `PlanResultDTO` gains two
  new fields: `path_ids: list[str]` and `blended: bool`. Existing fields
  (`success`, `trajectory`, `error_message`) are preserved.

### Established Patterns
- **Fail-fast on planning failure (D-16 from Phase 3):** If planning a group
  fails, the background thread should push a failed `PlanResultDTO` (with
  `success=False`) to the queue so the controller's generator loop can detect
  it and abort the goal immediately.
- **Lock pattern for single-goal enforcement:** `_executing_lock` and
  `_is_executing` in `URMovementController` are unchanged — one goal at a time
  remains enforced at `_goal_callback`.
- **Parameter declaration in `__init__`:** All parameters declared in `__init__`
  (not `on_configure`) to avoid `ParameterAlreadyDeclaredException` on
  re-configure. No new parameters expected in Phase 4 (blend radius comes from
  goal, not node parameters).

### Integration Points
- `_execute_callback` in `URMovementController` → replaces the
  `for group in groups: for path in group:` loop with:
  `plan_all(groups)` call + `for result in iterate_planned_trajectories():` loop.
- `ActionServer` in `on_configure` → add `cancel_callback=self._cancel_callback`
  parameter.
- `PlanResultDTO` `path_ids` field → feeds `fb.trajectory_path_ids` in feedback
  messages and `result.trajectory_paths_completed` in the final result.

</code_context>

<specifics>
## Specific Ideas

- The user described `iterate_planned_trajectories()` as acting like a generator
  that blocks on empty queue and terminates on `StopIteration` sentinel —
  implement this exactly as described, not as a callback or polling loop.
- The background thread checks `cancel_event` after each group is planned (not
  mid-plan). This avoids interrupting PILZ planning mid-call; it simply stops
  queuing further work after the current in-progress plan completes.
- `cancel()` must be callable from the `cancel_callback` without blocking the
  callback thread — `queue.Queue.queue.clear()` + `queue.put(StopIteration)` +
  `event.set()` is the expected pattern (all O(1) non-blocking operations).

</specifics>

<deferred>
## Deferred Ideas

- None — discussion stayed within Phase 4 scope.

</deferred>

---

*Phase: 4-Look-Ahead Planning & Blended Multi-Path Execution*
*Context gathered: 2026-05-29*
