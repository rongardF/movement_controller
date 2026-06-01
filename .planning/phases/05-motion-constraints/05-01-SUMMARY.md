---
phase: 5
plan: "05-01"
subsystem: motion-constraints
tags: [pydantic, dto, constraints, ros2-parameters, pilz]
dependency_graph:
  requires: []
  provides:
    - ConstraintConfigDTO (movement_controller.models)
    - 15 constraints.* ROS2 parameters in URMovementController
    - PilzPlannerService.set_constraints() stub
  affects:
    - src/movement_controller/movement_controller/models/
    - src/movement_controller/movement_controller/ur_movement_controller.py
    - src/movement_controller/movement_controller/services/pilz_planner_service.py
tech_stack:
  added: []
  patterns:
    - Pydantic v2 frozen model with model_validator(mode='after')
    - ROS2 declare_parameter with RosParameter.Type.STRING_ARRAY / DOUBLE_ARRAY for array types
    - try/except ValidationError → TransitionCallbackReturn.FAILURE in on_configure
key_files:
  created:
    - src/movement_controller/movement_controller/models/constraint_config_dto.py
  modified:
    - src/movement_controller/movement_controller/models/__init__.py
    - src/movement_controller/movement_controller/ur_movement_controller.py
    - src/movement_controller/movement_controller/services/pilz_planner_service.py
decisions:
  - ConstraintConfigDTO uses -1e9/+1e9 float sentinels (not float('inf')) since ROS2 float64 params don't support Python inf
  - workspace_enabled defined as NOT (all three axes span >= 2e9) per D-12
  - Array parameters declared with RosParameter.Type.STRING_ARRAY / DOUBLE_ARRAY to avoid rclpy empty-list type-inference problem
metrics:
  duration: "8 minutes"
  completed: "2026-06-01"
  tasks_completed: 2
  files_created: 1
  files_modified: 3
---

# Phase 5 Plan 01: ConstraintConfigDTO and Parameter Foundation Summary

**One-liner:** Frozen Pydantic v2 ConstraintConfigDTO with 15 ROS2 parameters, sentinel-based enable flags, and PilzPlannerService.set_constraints() stub.

## What Was Built

Created the constraint configuration foundation:

1. **`constraint_config_dto.py`** — Frozen Pydantic v2 model with 15 fields:
   - 6 workspace bounding box floats (sentinels ±1e9 = unconstrained)
   - 4 joint constraint lists (names, lower/upper limits, max velocities)
   - 3 orientation tolerance floats (default 2π = unconstrained)
   - 2 speed/acceleration caps (default 0.0 = unconstrained)
   - Two `@model_validator(mode='after')` methods: workspace bounds ordering + joint array length consistency
   - Three `@property` helpers: `workspace_enabled`, `joint_constraints_enabled`, `orientation_constraint_enabled`

2. **`models/__init__.py`** — Added `ConstraintConfigDTO` to imports and `__all__`

3. **`ur_movement_controller.py`** — Added:
   - `import math` and `from rclpy.parameter import Parameter as RosParameter`
   - `from movement_controller.models import TrajectoryGoalDTO, ConstraintConfigDTO`
   - `self._constraint_config: ConstraintConfigDTO | None = None` instance variable
   - 15 `declare_parameter` calls in `__init__` (6 workspace, 4 joint, 3 orientation, 2 speed)
   - Constraint parameter reading + `ConstraintConfigDTO` construction in `on_configure` wrapped in `try/except ValidationError → FAILURE`
   - `self._planner_service.set_constraints(dto)` call after successful validation

4. **`pilz_planner_service.py`** — Added:
   - `ConstraintConfigDTO` import
   - `self._constraint_config: ConstraintConfigDTO | None = None` instance variable
   - `def set_constraints(self, dto: ConstraintConfigDTO) -> None:` public method

## Verification Results

All plan acceptance criteria passed:
- `ConstraintConfigDTO()` (defaults) → `workspace_enabled=False`, `joint_constraints_enabled=False`, `orientation_constraint_enabled=False`
- `ConstraintConfigDTO(z_min=0.0, z_max=0.5)` → `workspace_enabled=True`
- `ConstraintConfigDTO(x_min=1.0, x_max=0.0)` → raises `ValidationError`
- `ConstraintConfigDTO(joint_names=['j1'], joint_lower_limits=[-1.0])` → raises `ValidationError`
- `ConstraintConfigDTO(orientation_tolerance_x=0.1)` → `orientation_constraint_enabled=True`
- 15 `constraints.*` parameters declared in `URMovementController.__init__`
- `PilzPlannerService.set_constraints()` present and stores dto

## Deviations from Plan

None - plan executed exactly as written.

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| T01+T02 | `7189add` | feat(05-01): add ConstraintConfigDTO and 15 constraint parameters |

## Known Stubs

- `PilzPlannerService.set_constraints()` stores `_constraint_config` but `_build_path_constraints()` does not yet exist. Plans 02–05 will wire this up.

## Self-Check: PASSED

- [x] `src/movement_controller/movement_controller/models/constraint_config_dto.py` exists
- [x] Commit `7189add` exists in git log
- [x] All 15 parameters declared (grep confirms)
- [x] `def set_constraints` present in pilz_planner_service.py
- [x] `ConstraintConfigDTO` exported from movement_controller.models
