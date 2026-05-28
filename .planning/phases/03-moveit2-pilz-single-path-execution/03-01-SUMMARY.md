---
phase: "03-moveit2-pilz-single-path-execution"
plan: "01"
subsystem: "controller / devcontainer"
tags: [moveit2, lifecycle, parameters, devcontainer]
key-files:
  modified:
    - docker/startup.sh
    - .devcontainer/devcontainer.json
    - src/movement_controller/movement_controller/ur_movement_controller.py
  created:
    - src/movement_controller/movement_controller/services/pilz_planner_service.py
decisions:
  - "Parameters (action_server_name, moveit_group_name, moveit_connection_timeout) all moved to __init__ to prevent ParameterAlreadyDeclaredException on re-configure"
  - "MoveItPy probe uses wait_for_service on /move_group/get_planning_scene — no daemon thread"
  - "PilzPlannerService stub created (plan 03-02 provides full implementation)"
metrics:
  completed: "2026-05-28"
---

# Phase 3 Plan 01: MoveItPy Lifecycle Wiring and Startup Summary

**One-liner:** MoveItPy wired into URMovementController lifecycle with configurable timeout probe and parameter declarations moved to __init__.

## Tasks Completed

### Task 1: Verify startup.sh and devcontainer.json (already done in discussion session)

Both files were already correct from the Phase 3 discussion session:
- `docker/startup.sh` — guards `rosdep init`, runs `rosdep update` + `rosdep install --from-paths src --ignore-src -r -y`, appends venv activation and ROS2 sourcing to `~/.bashrc`
- `.devcontainer/devcontainer.json` — `postCreateCommand` is exactly `"bash /workspaces/movement_controller/docker/startup.sh"`

### Task 2: Wire MoveItPy lifecycle into URMovementController

Four changes applied to `src/movement_controller/movement_controller/ur_movement_controller.py`:

1. **Imports added** — `from moveit.planning import MoveItPy`, `from moveit_msgs.srv import GetPlanningScene`, `from movement_controller.services.pilz_planner_service import PilzPlannerService`

2. **`__init__` extended** — added `self._moveit: MoveItPy | None = None` and `self._planner_service: PilzPlannerService | None = None`; moved the two existing `declare_parameter` calls (`action_server_name`, `moveit_group_name`) from `on_configure` to `__init__`; added new `declare_parameter('moveit_connection_timeout', 10.0, ...)`

3. **`on_configure` refactored** — removed `declare_parameter` calls (now in `__init__`); reads all parameters via `get_parameter()`; probes `move_group` via `wait_for_service` on `/move_group/get_planning_scene` with configurable timeout (returns `FAILURE` on timeout); calls `MoveItPy()` directly inside `try/except` (no daemon thread); creates `PilzPlannerService(self._moveit, planning_component)`; action server creation unchanged, still runs after MoveItPy init succeeds

4. **`on_cleanup` updated** — zeros both `self._moveit = None` and `self._planner_service = None` after action server teardown

**Also created:** `src/movement_controller/movement_controller/services/pilz_planner_service.py` — stub class with the correct constructor signature `(moveit, planning_component)` and a `plan()` method that raises `NotImplementedError` (full implementation in Plan 03-02).

## Files Modified

| File | Change |
|------|--------|
| `src/movement_controller/movement_controller/ur_movement_controller.py` | MoveItPy lifecycle wiring |
| `src/movement_controller/movement_controller/services/pilz_planner_service.py` | New stub service |

## Verification Results

```
$ grep 'rosdep install' docker/startup.sh
rosdep install --from-paths "${WORKSPACE}/src" --ignore-src -r -y

$ grep 'startup.sh' .devcontainer/devcontainer.json
  "postCreateCommand": "bash /workspaces/movement_controller/docker/startup.sh"

$ grep -n 'moveit_connection_timeout' src/movement_controller/movement_controller/ur_movement_controller.py
73:            'moveit_connection_timeout',
85:        timeout: float = self.get_parameter('moveit_connection_timeout').value

$ grep -n 'declare_parameter' src/movement_controller/movement_controller/ur_movement_controller.py
62:        self.declare_parameter(
67:        self.declare_parameter(
72:        self.declare_parameter(
# All three in __init__, zero in on_configure ✓

$ grep -n '_moveit = None\|_planner_service = None' src/movement_controller/movement_controller/ur_movement_controller.py
135:        self._moveit = None
136:        self._planner_service = None
# Both in on_cleanup ✓

All 36 unit tests: PASSED (0.35s)
```

## Deviations from Plan

**1. [Rule 2 - Missing critical functionality] Created PilzPlannerService stub**
- **Found during:** Task 2 — `from movement_controller.services.pilz_planner_service import PilzPlannerService` would fail at import time if the module didn't exist
- **Fix:** Created `services/pilz_planner_service.py` with the correct constructor signature `(moveit, planning_component)` and a stub `plan()` that raises `NotImplementedError`
- **Files modified:** `src/movement_controller/movement_controller/services/pilz_planner_service.py` (new)
- **Commit:** `20358f3`

## Self-Check: PASSED

- [x] `src/movement_controller/movement_controller/ur_movement_controller.py` exists
- [x] `src/movement_controller/movement_controller/services/pilz_planner_service.py` exists
- [x] Commit `20358f3` exists in git log
- [x] All acceptance criteria verified via grep
- [x] All 36 unit tests pass
