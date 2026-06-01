---
phase: 5
slug: motion-constraints
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-06-01
---

# Phase 5 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest + ament_pytest |
| **Config file** | `src/movement_controller/setup.cfg` (existing) |
| **Quick run command** | `python -m pytest src/movement_controller/tests/unit/test_constraint_config_dto.py -v` |
| **Full suite command** | `python -m pytest src/movement_controller/tests/ -v` |
| **Estimated runtime** | ~30 seconds |

---

## Sampling Rate

- **After every task commit:** Run `python -m pytest src/movement_controller/tests/unit/test_constraint_config_dto.py -v`
- **After every plan wave:** Run `python -m pytest src/movement_controller/tests/ -v`
- **Before `/gsd-verify-work`:** Full suite must be green
- **Max feedback latency:** 30 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 05-01-T01 | 01 | 1 | CON-01–06 | T-INV (bounds) | Pydantic rejects invalid bounds → on_configure returns FAILURE | unit | `pytest tests/unit/test_constraint_config_dto.py -v` | ❌ Wave 0 | ⬜ pending |
| 05-01-T02 | 01 | 1 | CON-04 | — | N/A | import | `python -c "from movement_controller.models import ConstraintConfigDTO; print('ok')"` | ❌ Wave 0 | ⬜ pending |
| 05-02-T01 | 02 | 2 | CON-01, CON-02, CON-03 | T-INV (geometry) | BOX full lengths; JointConstraint positive tolerances; identity quaternion reference | unit | `pytest tests/unit/test_pilz_planner_service.py -v` | ❌ Wave 0 | ⬜ pending |
| 05-02-T02 | 02 | 2 | CON-04 | T-CIRC | CIRC arc at position_constraints[0] preserved after merge | unit | `pytest tests/unit/test_pilz_planner_service.py::test_circ_merge_preserves_arc_constraint -v` | ❌ Wave 0 | ⬜ pending |
| 05-03-T01 | 03 | 2 | CON-05, CON-06 | T-INJ (speed) | Speed/accel cap rejects goal with exact D-07 error; velocity warning logged | unit | `pytest tests/unit/test_ur_movement_controller.py -v` | ❌ Wave 0 | ⬜ pending |
| 05-04-T01 | 04 | 3 | CON-01–06 | — | All constraint unit tests green | unit | `pytest tests/unit/test_constraint_config_dto.py tests/unit/test_pilz_planner_service.py tests/unit/test_ur_movement_controller.py -v` | ❌ Wave 0 | ⬜ pending |
| 05-04-T02 | 04 | 3 | CON-04 | — | Constraints injected in all MotionSequenceItems | unit | `pytest tests/unit/test_pilz_planner_service.py::test_constraints_injected_in_all_items -v` | ❌ Wave 0 | ⬜ pending |
| 05-05-T01 | 05 | 3 | CON-01 | — | Integration: workspace violation causes success=False result | integration | `pytest tests/integration/test_integration_ur_movement_controller.py::test_planning_fails_when_goal_outside_workspace -v` | ❌ Wave 0 | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

All test files are created by Wave 3 plans (05-04, 05-05). Wave 3 is the Wave 0 for test infrastructure.

- [ ] `src/movement_controller/tests/unit/test_constraint_config_dto.py` — new file covering CON-01 through CON-06 (created by Plan 05-04)
- [ ] `src/movement_controller/tests/unit/test_pilz_planner_service.py` — extended with constraint tests (updated by Plan 05-04)
- [ ] `src/movement_controller/tests/unit/test_ur_movement_controller.py` — extended with speed cap tests (updated by Plan 05-04)
- [ ] `src/movement_controller/tests/integration/test_integration_ur_movement_controller.py` — extended with workspace violation test (updated by Plan 05-05)

> **Note:** Waves 1 and 2 (Plans 01–03) are implementation waves. Test coverage for those waves is
> provided by Wave 3 (Plans 04–05). This is an explicit delayed Nyquist coverage pattern — Wave 2
> tasks (05-02-T01, 05-02-T02, 05-03-T01) have smoke-test `<acceptance_criteria>` using source
> assertions and type-level checks; full automated coverage arrives in Wave 3.

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| `ValidateSolution` actually enforces workspace bounding box against real PILZ planning | CON-01 | Requires live `move_group` with PILZ — not available without full sim stack | Launch sim, configure z_max=0.5, send goal at z=2.0, assert planning failure |
| Per-joint velocity limits actually enforced by PILZ (D-13 limitation) | CON-06 | PILZ has no per-joint velocity API; enforcement is via URDF/joint_limits.yaml at startup | Manual acceptance test in Phase 8 (real hardware) |

---

## Requirement Coverage Map

| Req ID | Behavior to Test | Plan | Test File | Test Name |
|--------|-----------------|------|-----------|-----------|
| CON-01 | Workspace BOX PositionConstraint dimensions (full lengths, midpoint center) | 02, 04 | `test_constraint_config_dto.py` | `test_position_constraint_box_dimensions` |
| CON-01 | `workspace_enabled` returns False at defaults, True when narrowed | 01, 04 | `test_constraint_config_dto.py` | `test_workspace_disabled_when_all_at_defaults`, `test_workspace_enabled_when_x_narrowed` |
| CON-02 | JointConstraint midpoint position, positive tolerance_below | 02, 04 | `test_constraint_config_dto.py` | `test_joint_constraint_position_is_midpoint` |
| CON-03 | OrientationConstraint tolerance fields, parameterization=0, identity ref | 02, 04 | `test_constraint_config_dto.py` | `test_orientation_constraint_fields` |
| CON-04 | `set_constraints()` stores DTO; constraints appear in every MotionSequenceItem | 02, 04 | `test_pilz_planner_service.py` | `test_constraints_injected_in_all_items` |
| CON-04 | CIRC merge: arc at `position_constraints[0]`, BOX appended at `[1:]` | 02, 04 | `test_pilz_planner_service.py` | `test_circ_merge_preserves_arc_constraint` |
| CON-05 | Goal rejected with D-07 error when cartesian_speed > max_cartesian_speed | 03, 04 | `test_ur_movement_controller.py` | `test_goal_rejected_speed_exceeded` |
| CON-05 | Goal rejected for acceleration violation (same format) | 03, 04 | `test_ur_movement_controller.py` | `test_goal_rejected_acceleration_exceeded` |
| CON-06 | `joint_max_velocities` stored in DTO; WARNING logged; not injected | 02, 04 | `test_constraint_config_dto.py` | `test_joint_velocity_warning_logged` |
| CON-01 (integration) | Workspace bounding-box violation causes success=False | 05 | `test_integration_ur_movement_controller.py` | `test_planning_fails_when_goal_outside_workspace` |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: Wave 2 implementation tasks have source-assertion acceptance criteria + full coverage in Wave 3
- [ ] Wave 0 (Wave 3 tests) covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 30s
- [ ] `nyquist_compliant: true` set in frontmatter (set after Wave 3 green)

**Approval:** pending
