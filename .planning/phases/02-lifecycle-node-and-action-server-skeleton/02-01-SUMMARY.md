---
id: "02-01"
phase: 2
status: completed
completed: 2026-05-27
---

# Summary — 02-01: LifecycleNode Base — URMovementController Skeleton

## What was built

Created `src/movement_controller/movement_controller/ur_movement_controller.py` containing the `URMovementController` class — a `rclpy.lifecycle.LifecycleNode` subclass — plus a `main()` entry point.

## Files modified

| File | Change |
|------|--------|
| `src/movement_controller/movement_controller/ur_movement_controller.py` | Created — URMovementController with full lifecycle callbacks and parameter declarations |

## Key decisions

- `URMovementController(LifecycleNode)` with `node_name='ur_movement_controller'` default
- `__init__` initialises `_action_server=None`, `_is_executing=False`, `_executing_lock=threading.Lock()`
- `on_configure` declares `action_server_name` and `moveit_group_name` parameters with `ParameterDescriptor`
- All lifecycle callbacks return `TransitionCallbackReturn.SUCCESS`
- `on_cleanup` destroys `_action_server` if not None
- Action Server and callbacks wired in the same file (Plan 02-02 extended this)
- `ServerGoalHandle` from `rclpy.action.server` (not `rclpy.action`) — Jazzy API difference

## Verification

- `python3 -c "from movement_controller.ur_movement_controller import URMovementController"` passes
- `grep "class URMovementController(LifecycleNode)"` returns match
- `grep -c "TransitionCallbackReturn.SUCCESS"` returns 4

## Notes

Fixed import: `ServerGoalHandle` must be imported from `rclpy.action.server`, not `rclpy.action` (which only exports `ActionServer`, `GoalResponse`, `CancelResponse` in Jazzy).
