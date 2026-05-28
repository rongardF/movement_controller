# Phase 3: MoveIt2 + PILZ Single-Path Execution — Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-28
**Phase:** 3-MoveIt2-PILZ-Single-Path-Execution
**Areas discussed:** MoveItPy ownership & lifecycle, CIRC path planning approach,
Multi-path group behavior in Phase 3, Planning failure strategy

---

## MoveItPy Ownership & Lifecycle

| Option | Description | Selected |
|--------|-------------|----------|
| Controller creates MoveItPy, injects component into service | on_configure creates MoveItPy, gets planning component, passes it to PilzPlannerService. Service is stateless. | ✓ |
| Service creates MoveItPy internally | PilzPlannerService creates its own MoveItPy instance; fully self-contained. | |

**User's choice:** Controller creates MoveItPy, injects planning_component into service.

| Timeout option | Description | Selected |
|----------------|-------------|----------|
| 10 seconds | Fixed default | |
| 30 seconds | More generous for real hardware | |
| Configurable parameter | `moveit_connection_timeout` node parameter | ✓ |

**User's choice:** Configurable parameter (`moveit_connection_timeout`, float, default 10.0s).

| Timeout failure | Description | Selected |
|-----------------|-------------|----------|
| Fail on_configure → FAILURE | Log error, return FAILURE. Node stays UNCONFIGURED. | ✓ |
| Warn but continue | Allow configure to succeed; defer error to first planning call. | |

**User's choice:** Fail on_configure with FAILURE return.

| Teardown option | Description | Selected |
|-----------------|-------------|----------|
| Destroy in on_cleanup, recreate in on_configure | Mirrors action_server lifecycle pattern from Phase 2. | ✓ |
| Just nullify reference | Simpler code, let GC handle it. | |

**User's choice:** Destroy in on_cleanup, recreate in on_configure.

**Notes:** User asked for clarification on what MoveItPy is and whether it creates
a MoveGroup node. Explanation given: MoveItPy connects to an already-running
move_group node (does NOT create one). In on_configure: `self._moveit = MoveItPy(...)`.
`self._planning_component = self._moveit.get_planning_component('ur_manipulator')`.
After clarification user confirmed: service receives `planning_component` (stateless),
not the full `MoveItPy` instance.

---

## CIRC Path Planning Approach

| Option | Description | Selected |
|--------|-------------|----------|
| Support both interim and center modes | Both circ_type values handled in Phase 3. | ✓ |
| Interim only in Phase 3 | Defer center mode. | |

**User's choice:** Support both interim and center modes.

| Invalid circ_type | Description | Selected |
|-------------------|-------------|----------|
| Reject in goal_callback (fail-fast) | Empty/unrecognized circ_type on CIRC path → reject before accept. | ✓ |
| Default to interim mode | Treat missing circ_type as 'interim'. | |

**User's choice:** Reject in goal_callback.

**Notes:** `circ_type` and `circ_point` fields are already defined in `TrajectoryPath.msg`
from Phase 1 (D-01/D-02). PILZ CIRC constraint API must be researched before
implementation — exact Python API for passing interim/center to moveit_py CIRC
planner is not guessed here.

---

## Multi-Path Group Behavior in Phase 3

| Option | Description | Selected |
|--------|-------------|----------|
| Execute each path individually (non-blended fallback) | Flatten multi-path groups to single paths in execute_callback. | ✓ |
| Reject goals with blend groups | Force callers to set blend_radius=0 until Phase 4. | |

**User's choice:** Non-blended fallback — execute each path individually.

| Flatten approach | Description | Selected |
|-----------------|-------------|----------|
| Flatten in execute_callback (keep grouper output intact) | execute_callback unrolls groups. Grouper unchanged for Phase 4. | ✓ |
| Re-flatten after grouper | Semantically equivalent variant. | |

**User's choice:** Flatten in execute_callback, keeping grouper output intact.

| Feedback for flattened paths | Description | Selected |
|-----------------------------|-------------|----------|
| Same feedback sequence (executing → completed per path) | Consistent with Phase 2 stub contract. | ✓ |
| Extra 'blending_skipped' status | Signal when stop-start happens. | |

**User's choice:** Same feedback sequence.

**Notes:** Phase 4 will use grouper output directly with MoveGroupSequence.
Phase 3 intentionally produces stop-start for blend groups so callers don't
need to change goals between Phase 3 and Phase 4.

---

## Planning Failure Strategy

| Option | Description | Selected |
|--------|-------------|----------|
| Fail-fast: abort whole goal on first failure | Abort immediately. success=False, error_message with path_id. | ✓ |
| Attempt all, report failed paths | Useful for diagnostics but complicates result handling. | |

**User's choice:** Fail-fast.

| Execution error | Description | Selected |
|-----------------|-------------|----------|
| Abort on execution error, no retry | Abort goal with path_id and error in error_message. | ✓ |
| Retry once | Retry before aborting. | |

**User's choice:** Abort on execution error, no retry.

**Notes:** Both planning and execution failures log at ERROR level before returning.
Consistent with Phase 2 error handling (D-18).

---

## Agent's Discretion

- CIRC validation placement: either in `TrajectoryPathDTO` validator or in
  `goal_callback`. User deferred this to researcher/planner to decide based on
  moveit_py CIRC API findings.

## Deferred Ideas

- Action server cancellation handling — still deferred from Phase 2; becomes
  meaningful in Phase 3/4 but not in Phase 3 scope.
- Retry on planning failure — discussed and rejected in favor of fail-fast.
- `blending_skipped` feedback status — discussed and rejected; not needed until
  Phase 4 ships blended execution.
