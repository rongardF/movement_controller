---
phase: 01-package-scaffold-and-interface-definitions
plan: 02
subsystem: api
tags: [rosidl, ros2-interfaces, action, msg, srv, pilz, trajectory, scene-management]

requires:
  - phase: 01-01
    provides: rosidl_generate_interfaces wired in CMakeLists.txt

provides:
  - ExecuteTrajectory action interface (Goal/Result/Feedback with TrajectoryPath[] goal)
  - TrajectoryPath message with MOTION_TYPE/CIRC_TYPE constants and all CIRC fields
  - 5 scene management service interfaces (AddObject, AttachObject, DetachObject, RemoveObject, ModifyAcm)

affects: [02, 03, 04, 05, 06, all subsequent phases]

tech-stack:
  added: [rosidl action/msg/srv interface files]
  patterns:
    - BSD-3-Clause license header on all interface files
    - CIRC fields embedded flat in TrajectoryPath.msg (no nested type)
    - Consistent service response pattern (bool success + string error_message + string object_id)

key-files:
  created:
    - src/movement_controller/action/ExecuteTrajectory.action
    - src/movement_controller/msg/TrajectoryPath.msg
    - src/movement_controller/srv/AddObject.srv
    - src/movement_controller/srv/AttachObject.srv
    - src/movement_controller/srv/DetachObject.srv
    - src/movement_controller/srv/RemoveObject.srv
    - src/movement_controller/srv/ModifyAcm.srv
  modified: []

key-decisions:
  - "CIRC fields embedded flat in TrajectoryPath.msg rather than a nested type, keeping the msg self-contained"
  - "Server-generated UUID4 for object IDs in AddObject — client does not provide ID"
  - "Consistent bool success + string error_message + string object_id response for all scene services"
  - "MOTION_TYPE_* and CIRC_TYPE_* as string constants in TrajectoryPath.msg for rosidl compatibility"

patterns-established:
  - "Interface-first design: all ROS2 message/action/service types defined before any Python implementation"
  - "String constants in .msg files used for enum-like values (rosidl does not support native enums)"

requirements-completed: [PKG-03, PKG-05, UR-02]

duration: 6min
completed: 2026-05-27
---

# Plan 01-02: ROS2 Interface Files

**Defined all 7 ROS2 interfaces (1 action, 1 message, 5 services) for trajectory execution and scene management, establishing the contract every subsequent phase implements against.**

## Performance

- **Duration:** ~6 min
- **Completed:** 2026-05-27
- **Tasks:** 7 interface files
- **Files modified:** 7

## Accomplishments
- Created `ExecuteTrajectory.action` with `TrajectoryPath[] paths` goal, `success/error_message/trajectory_paths_completed` result, and `status/trajectory_path_ids` feedback
- Created `TrajectoryPath.msg` with `MOTION_TYPE_LIN/PTP/CIRC` and `CIRC_TYPE_INTERIM/CENTER` string constants, all path fields, and flat CIRC fields (`circ_point`, `circ_type`, `circ_point_frame_id`)
- Created all 5 scene service files with consistent `bool success + string error_message + string object_id` response
- All 7 files carry BSD-3-Clause license headers

## Task Commits

1. **Wave 2: All 7 ROS2 interface files** - `081a962` (feat(1-2): add ROS2 interface files)

## Files Created/Modified
- `src/movement_controller/action/ExecuteTrajectory.action` — Action interface for trajectory execution
- `src/movement_controller/msg/TrajectoryPath.msg` — Message type with motion constants and CIRC fields
- `src/movement_controller/srv/AddObject.srv` — Add collision object (primitive or mesh)
- `src/movement_controller/srv/AttachObject.srv` — Attach collision object to robot link
- `src/movement_controller/srv/DetachObject.srv` — Detach attached collision object
- `src/movement_controller/srv/RemoveObject.srv` — Remove collision object from scene
- `src/movement_controller/srv/ModifyAcm.srv` — Modify Allowed Collision Matrix entries

## Decisions Made
- **Flat CIRC fields in TrajectoryPath.msg**: avoids a nested rosidl type and keeps the message self-contained. The `circ_point`, `circ_type`, and `circ_point_frame_id` fields are simply ignored for non-CIRC paths.
- **Server-generated UUID4 for AddObject**: the service returns the generated `object_id` in the response rather than requiring the client to provide one, preventing ID collisions.

## Deviations from Plan
None — plan executed exactly as written.
