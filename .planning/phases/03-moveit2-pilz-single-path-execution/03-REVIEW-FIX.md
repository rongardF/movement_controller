---
phase: "03"
fixed_at: "2026-05-28T00:00:00Z"
review_path: .planning/phases/03-moveit2-pilz-single-path-execution/03-REVIEW.md
iteration: 1
findings_in_scope: 6
fixed: 6
skipped: 0
status: all_fixed
---

# Phase 03: Code Review Fix Report

**Fixed at:** 2026-05-28T00:00:00Z
**Source review:** .planning/phases/03-moveit2-pilz-single-path-execution/03-REVIEW.md
**Iteration:** 1

**Summary:**
- Findings in scope: 6
- Fixed: 6
- Skipped: 0

## Fixed Issues

### WR-001: Non-idempotent `.bashrc` modification in `startup.sh`

**Files modified:** `docker/startup.sh`
**Commit:** 56b9483
**Applied fix:** Replaced bare `echo ... >> ~/.bashrc` appends with `grep -qF ... || echo ... >> ~/.bashrc` guards so the script is idempotent on repeated runs.

### WR-002: Unresolved FIXME — invalid `circ_type` on non-CIRC paths raises `ValueError`

**Files modified:** `src/movement_controller/movement_controller/models/trajectory_path_dto.py`
**Commit:** 13df8a0
**Applied fix:** Replaced the one-liner with a try/except block in the `else` branch of `from_ros_msg`. A `ValueError` from `CircTypeEnum(ros_msg.circ_type)` now silently defaults to `CircTypeEnum.INTERIM` (circ_type is irrelevant for LIN/PTP paths). Removed the `# FIXME` comment.

### WR-003: `set_path_constraints` called outside the `try/finally` block

**Files modified:** `src/movement_controller/movement_controller/services/pilz_planner_service.py`
**Commit:** 6379e8b
**Applied fix:** Moved the `set_path_constraints` call inside the `try` block so the `finally` clause always clears constraints even if `set_path_constraints` itself raises.

### WR-004: `_is_executing` deactivation "wind-down signal" is a dead write  +  WR-005: `goal_handle.abort()` can trigger double-abort via outer `except`

**Files modified:** `src/movement_controller/movement_controller/ur_movement_controller.py`
**Commit:** 8161001
**Applied fix (WR-004 Part A):** Removed the `with self._executing_lock: self._is_executing = False` block from `on_deactivate`. The `_is_executing` flag is already cleared by the `finally` block in `_execute_callback`; the `_is_active = False` assignment is sufficient to block new goals.
**Applied fix (WR-004 Part B):** Added `if not self._is_active` guard at the top of each per-path iteration loop to enable graceful halt on mid-trajectory deactivation. The abort call inside this guard is also wrapped in try/except (covers WR-005 for this path).
**Applied fix (WR-005):** Wrapped all four `goal_handle.abort()` call sites in `try/except Exception: pass` to swallow `ActionError` when the goal handle is already in a terminal state due to concurrent client cancellation.

### WR-006: Integration tests mutate shared fixture state with manual (fragile) restoration

**Files modified:** `src/movement_controller/tests/integration/test_moveit_execution_integration.py`
**Commit:** 64a6e54
**Applied fix:** Added `monkeypatch` parameter to `test_execute_trajectory_aborts_on_plan_failure` and `test_execute_trajectory_aborts_on_execution_failure`. Replaced direct attribute assignment + manual restore with `monkeypatch.setattr(...)` so mock state is auto-restored after each test. Removed the manual restore lines.

## Skipped Issues

None — all findings were fixed.

---

**Test results after all fixes:**  
`57 passed in 0.47s` — full test suite passes with no regressions.

---

_Fixed: 2026-05-28T00:00:00Z_
_Fixer: the agent (gsd-code-fixer)_
_Iteration: 1_
