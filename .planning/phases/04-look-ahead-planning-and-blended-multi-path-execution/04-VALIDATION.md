---
phase: 4
slug: look-ahead-planning-and-blended-multi-path-execution
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-29
---

# Phase 4 Validation — Look-Ahead Planning & Blended Multi-Path Execution

## Quick Run

```bash
cd /workspaces/movement_controller
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python -m pytest src/movement_controller/tests/ -v
```

## Full Suite

```bash
cd /workspaces/movement_controller
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select movement_controller && colcon test-result --verbose
```

---

## Per-Task Verification Map

Derived from RESEARCH.md "Phase Requirements → Test Map".

| Req ID | Behavior Under Test | Test Type | File | Test Name / Filter | Notes |
|--------|---------------------|-----------|------|--------------------|-------|
| MOT-02 | `MotionSequenceRequest` built for all groups | Unit | `tests/unit/test_pilz_planner_service.py` | `-k sequence_request` | Mock service client |
| MOT-02 | `blend_radius` propagated to `MotionSequenceItem.blend_radius` | Unit | `tests/unit/test_pilz_planner_service.py` | `-k blend_radius` | Assert item.blend_radius == path.blend_radius |
| MOT-02 | Last item's blend_radius is always 0 | Unit | `tests/unit/test_pilz_planner_service.py` | `-k last_item` | Pitfall 1 guard |
| MOT-03 | Look-ahead: group N+1 planned before group N execute_and_wait returns | Unit | `tests/unit/test_pilz_planner_service.py` | `test_look_ahead_plans_next_group_before_current_finishes` | Temporal overlap assertion via queue inspection |
| MOT-04 | Generator yields immediately when queue pre-populated | Unit | `tests/unit/test_pilz_planner_service.py` | `-k queue` | Put items in queue, assert yield is instant |
| MOT-04 | No re-plan latency: `execute_and_wait` called immediately after dequeue | Integration | `tests/integration/test_moveit_execution_integration.py` | `test_execute_3_path_blended_trajectory_success` | Verifies plan_all called once before any execution |
| D-06 | `PlanResultDTO.path_ids` contains correct IDs | Unit | `tests/unit/test_pilz_planner_service.py` | `test_iterate_yields_single_path_result` | Assert path_ids == [uuid] |
| D-06 | `PlanResultDTO.blended` is True for 2+ path group | Unit | `tests/unit/test_pilz_planner_service.py` | `test_iterate_blended_group_sets_blended_true` | Assert blended=True |
| D-09 | `cancel()` drains queue and pushes sentinel | Unit | `tests/unit/test_pilz_planner_service.py` | `test_cancel_terminates_iterator_cleanly` | Thread cancels mid-planning |
| D-09 | Generator terminates after cancel() | Unit | `tests/unit/test_pilz_planner_service.py` | `test_cancel_terminates_iterator_cleanly` | list() returns within 2s |
| D-10 | `cancel_callback` returns in < 10ms | Unit | `tests/unit/test_pilz_planner_service.py` | `test_plan_all_starts_background_thread` | Implicit — cancel is non-blocking |
| D-01 | Group-level feedback: one 'executing' + one 'completed' per group | Integration | `tests/integration/test_moveit_execution_integration.py` | `test_execute_trajectory_feedback_order` | Assert status sequence |
| D-02 | `trajectory_paths_completed` is flat list of all IDs | Integration | `tests/integration/test_moveit_execution_integration.py` | `test_execute_3_path_blended_trajectory_success` | 3 paths → completed=['id1','id2','id3'] |
| D-04 | Fresh queue/event per `plan_all()` call | Unit | `tests/unit/test_pilz_planner_service.py` | `test_plan_all_creates_fresh_queue_per_call` | q1 is not q2 |
| D-07 | All groups (including single-path) use MotionSequenceRequest | Unit | `tests/unit/test_pilz_planner_service.py` | `test_iterate_yields_single_path_result` | Uniform code path verified |

---

## Wave 0 Requirements

These test files must exist before Phase 4 implementation tasks can produce passing tests.
Status: **[ ] incomplete** until Plan 04-04 is executed.

- [ ] `src/movement_controller/tests/unit/test_pilz_planner_service.py`
  — Covers: MOT-02, MOT-03, MOT-04, D-04, D-06, D-09 (mock `GetMotionSequence` service client)
  — New test functions to add: `test_plan_all_starts_background_thread`, `test_iterate_yields_single_path_result`, `test_iterate_blended_group_sets_blended_true`, `test_last_item_blend_radius_forced_to_zero`, `test_cancel_terminates_iterator_cleanly`, `test_planning_failure_yields_error_dto`, `test_plan_all_creates_fresh_queue_per_call`, `test_look_ahead_plans_next_group_before_current_finishes`

- [ ] `src/movement_controller/tests/integration/test_moveit_execution_integration.py`
  — Covers: D-01, D-02, MOT-03/04 integration path
  — Existing tests must be updated to use `plan_all` + `iterate_planned_trajectories` API
  — New tests to add: `test_execute_3_path_blended_trajectory_success`, `test_cancel_during_execution_returns_canceled`

---

## Nyquist Compliance Checklist

- [ ] Every requirement ID (MOT-02, MOT-03, MOT-04) has at least one automated test
- [ ] Every decision (D-01 through D-10) has a verification entry in the map above
- [ ] Look-ahead temporal overlap (MOT-03) has a timing assertion test, not just a log check
- [ ] All new tests pass: `python -m pytest src/movement_controller/tests/ -v`
- [ ] No regressions: existing Phase 3 tests still pass
- [ ] `colcon test --packages-select movement_controller` exits 0

Update `nyquist_compliant: true` and `wave_0_complete: true` in frontmatter after all items above are checked.
