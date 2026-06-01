# Summary: 05-03 — Speed/Acceleration Cap Enforcement in _goal_callback

**Phase:** 5 — Motion Constraints  
**Plan:** 05-03  
**Status:** Complete  
**Commit:** 9409c3d

## What Was Built

Added per-path speed and acceleration cap enforcement to `URMovementController._goal_callback`.

The new block (after DTO validation, before `return GoalResponse.ACCEPT`):

1. Guards with `if self._constraint_config is not None:` — no-op if node not configured
2. Iterates over `self._trajectory_goal.paths` (already-validated `TrajectoryPathDTO` objects)
3. **Cartesian speed check**: if `path.cartesian_speed > 0.0` AND `max_cartesian_speed > 0.0` AND `path.cartesian_speed > max_cartesian_speed`:
   - Logs D-07-format error: `"Path '{path_id}' cartesian_speed {value} m/s exceeds node maximum {limit} m/s (constraints.max_cartesian_speed)"`
   - Resets `_is_executing = False` and `_trajectory_goal = None`
   - Returns `GoalResponse.REJECT`
4. **Acceleration check**: same structure, with `acceleration` field and `constraints.max_acceleration` label

The `return GoalResponse.ACCEPT` remains as the final statement in `_goal_callback`.

## Files Modified

- `src/movement_controller/movement_controller/ur_movement_controller.py`

## Verification Results

```
Speed cap block at lines 331-362
_constraint_config guard at line 331
GoalResponse.ACCEPT at line 364
D-07 message format with (constraints.max_cartesian_speed) and (constraints.max_acceleration)
Both rejection branches reset _is_executing = False
```

## Deviations from Plan

None — plan executed exactly as written.

## Self-Check: PASSED

All acceptance criteria verified:
- ✅ `_goal_callback` contains `if self._constraint_config is not None:` guard before speed loop
- ✅ Cartesian speed error message contains `'cartesian_speed'` and `'(constraints.max_cartesian_speed)'`
- ✅ Acceleration error message contains `'acceleration'` and `'(constraints.max_acceleration)'`
- ✅ Both rejection branches set `self._trajectory_goal = None` and reset `self._is_executing = False` inside `with self._executing_lock:`
- ✅ Final `return GoalResponse.ACCEPT` still present after the new block
