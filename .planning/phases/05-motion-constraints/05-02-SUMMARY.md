# Summary: 05-02 — PilzPlannerService Constraint Building

**Phase:** 5 — Motion Constraints  
**Plan:** 05-02  
**Status:** Complete  
**Commit:** 4edec51

## What Was Built

Added all constraint-building logic to `PilzPlannerService`:

1. **`_build_path_constraints(tool_frame: str) -> Constraints`** — Constructs a `moveit_msgs/Constraints` message from the active `ConstraintConfigDTO`:
   - Workspace BOX: `SolidPrimitive.BOX` with full lengths (`x_max-x_min`, etc.) and center midpoint, only when `cfg.workspace_enabled`
   - Joint constraints: `JointConstraint` per joint with midpoint position and symmetric tolerances, only when `cfg.joint_constraints_enabled`
   - Orientation constraint: identity quaternion (`w=1.0`), `parameterization=0` (XYZ_EULER_ANGLES), only when `cfg.orientation_constraint_enabled`
   - Returns empty `Constraints()` when `_constraint_config is None`

2. **`_merge_circ_and_path_constraints(circ, path) -> Constraints`** — Merges CIRC arc definition with path constraints. Preserves `circ.position_constraints[0]` (the arc point) at index 0; appends workspace BOX from `path.position_constraints` at `[1:]`. Preserves `circ.name` ('interim' or 'center') so PILZ reads the correct CIRC type.

3. **Updated `_generate_motion_sequence_request()`**:
   - Replaced hardcoded `= 1.0` scaling factors with dynamic expressions from `path_dto` (D-06):
     - `max(0.01, min(1.0, path_dto.cartesian_speed)) if path_dto.cartesian_speed > 0.0 else 1.0`
     - `max(0.01, min(1.0, path_dto.acceleration)) if path_dto.acceleration > 0.0 else 1.0`
   - Every item now calls `_build_path_constraints()` unconditionally
   - CIRC items merge arc + path constraints via `_merge_circ_and_path_constraints()`
   - LIN/PTP items set `item.req.path_constraints = path_constraints` directly

4. **`set_constraints()` WARNING** — Added `logger.warning()` when `dto.joint_max_velocities` is non-empty, noting PILZ has no per-joint velocity API (D-13).

## Files Modified

- `src/movement_controller/movement_controller/services/pilz_planner_service.py`

## Verification Results

```
JointConstraint import present at line 44
_build_path_constraints defined at line 125
_merge_circ_and_path_constraints defined at line 182
Dynamic scaling at lines 246-247 (no hardcoded = 1.0)
_build_path_constraints called in _generate_motion_sequence_request at line 257
_merge_circ_and_path_constraints called at line 260
Warning log at lines 380-382
```

## Deviations from Plan

None — plan executed exactly as written.

## Self-Check: PASSED

All acceptance criteria verified:
- ✅ `JointConstraint` in `from moveit_msgs.msg import (...)` block
- ✅ `def _build_path_constraints(self, tool_frame: str) -> Constraints:` exists in private methods region
- ✅ `box.dimensions = [cfg.x_max - cfg.x_min, cfg.y_max - cfg.y_min, cfg.z_max - cfg.z_min]` (full lengths)
- ✅ `jc.position = (lower + upper) / 2.0` and both `tolerance_above`/`tolerance_below` computed from midpoint
- ✅ `oc.parameterization = 0` present in orientation branch
- ✅ `oc.orientation.w = 1.0` (identity quaternion reference)
- ✅ `set_constraints()` logs WARNING when `dto.joint_max_velocities` is non-empty
- ✅ `_generate_motion_sequence_request` no longer contains hardcoded `= 1.0` for scaling (uses dynamic expression)
- ✅ `_generate_motion_sequence_request` calls `_build_path_constraints` on every loop iteration
- ✅ `_generate_motion_sequence_request` has `_merge_circ_and_path_constraints` in CIRC branch
- ✅ `_generate_motion_sequence_request` has `item.req.path_constraints = path_constraints` in else branch
