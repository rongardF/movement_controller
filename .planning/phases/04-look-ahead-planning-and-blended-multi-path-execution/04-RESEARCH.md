# Phase 4: Look-Ahead Planning & Blended Multi-Path Execution — Research

**Researched:** 2026-05-29
**Domain:** MoveIt2 PILZ sequence planning, Python threading, ROS2 ActionServer cancellation
**Confidence:** HIGH (all critical API claims verified by Python introspection and official docs)

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **D-01:** Feedback is **group-level** — one `executing` and one `completed` message per blend group.
- **D-02:** `result.trajectory_paths_completed` is a flat list of all completed IDs; partial completion on failure.
- **D-03:** Background planning thread and queue live **inside `PilzPlannerService`** — not in a new class.
- **D-04:** `PilzPlannerService.plan_all(groups)` starts a **fresh** thread + `queue.Queue` per goal invocation.
- **D-05:** `iterate_planned_trajectories()` is a **generator** that yields `PlanResultDTO`, blocks on empty, terminates on `StopIteration` sentinel.
- **D-06:** `PlanResultDTO` gains `path_ids: list[str]` and `blended: bool` fields.
- **D-07:** **All groups** (including single-path, blend_radius=0) are planned via `MotionSequenceRequest`. Uniform code path.
- **D-08:** State propagation — chain `last_predicted_state` from end of group N as `start_state` for group N+1. Single variable.
- **D-09:** `cancel()` — sets `threading.Event`, clears queue (thread-safe), pushes `StopIteration` sentinel.
- **D-10:** Non-blocking `cancel_callback` on ActionServer — calls `planner_service.cancel()` and returns immediately.

### Agent's Discretion
- How to convert `moveit_msgs/RobotTrajectory` → `moveit.core.robot_trajectory.RobotTrajectory` for execution.
- Service vs. action client for background planning (researched below).
- Exact utility function signature for building `goal_constraints` from `PoseStamped`.
- Whether to expose cancellation event check inside the execution loop in `_execute_callback`.

### Deferred Ideas (OUT OF SCOPE)
- Motion constraints (Phase 5)
- Scene management (Phase 6)
- Launch files (Phase 7)
- Real hardware validation (Phase 8)
- Per-move constraint overrides
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| MOT-02 | Multi-path blended trajectories via MoveIt2 `MoveGroupSequence` action | §API Finding 1: confirmed `/sequence_move_group` action and `/plan_sequence_path` service both available |
| MOT-03 | Look-ahead planning: groups N+1, N+2 planned on background thread while group N executes | §Architecture Pattern 1: `GetMotionSequence` service callable from background thread via `call_async()` |
| MOT-04 | Planned trajectories queued; next group executes immediately (no re-plan latency) | §Architecture Pattern 2: `queue.Queue` → pre-planned `moveit_msgs/RobotTrajectory` stored in `PlanResultDTO` |
</phase_requirements>

---

## Summary

Phase 4 expands `PilzPlannerService` to own a background planning thread and `queue.Queue`, uses `MotionSequenceRequest` for all groups (via the `GetMotionSequence` ROS2 service at `/plan_sequence_path`), chains end-state from plan N as start-state for plan N+1, and wires a generator-based consumption loop into `URMovementController._execute_callback`.

The critical finding is that **`moveit_py` exposes no `plan_sequence()` method** — sequence planning requires a raw ROS2 service client to `/plan_sequence_path` (`moveit_msgs.srv.GetMotionSequence`). For execution of the planned blended segments, each `moveit_msgs/RobotTrajectory` from the service response is converted to `moveit.core.robot_trajectory.RobotTrajectory` (via `set_robot_trajectory_msg`) and submitted to `TrajectoryExecutionManager.push()` + `execute_and_wait()`. A **critical prerequisite** is that the `move_group` node must be launched with the `pilz_industrial_motion_planner/MoveGroupSequenceService` capability enabled.

**Primary recommendation:** Add `node` parameter to `PilzPlannerService` constructor; create a `GetMotionSequence` service client; implement `plan_all()` background thread that calls the service per group and queues `PlanResultDTO(path_ids, blended, trajectories=[moveit_msgs/RobotTrajectory])`; implement `execute()` in the controller using `TrajectoryExecutionManager` for multi-segment blended execution.

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Sequence planning per group | `PilzPlannerService` (background thread) | `move_group` node (PILZ plugin) | D-03: thread lives inside service |
| Queue / look-ahead state | `PilzPlannerService` | — | D-03/D-04 |
| State propagation (end→start) | `PilzPlannerService` (background thread) | — | D-08 |
| Cancellation coordination | `PilzPlannerService.cancel()` + ActionServer `cancel_callback` | — | D-09/D-10 |
| Trajectory execution | `URMovementController._execute_callback` | `TrajectoryExecutionManager` | Controller drives execution |
| Group-level feedback | `URMovementController._execute_callback` | — | D-01 |

---

## API Findings

### Finding 1 — moveit_py has NO `plan_sequence()` method [VERIFIED: Python introspection]

```python
import moveit.planning as mp
# MoveItPy methods: execute, get_planning_component, get_planning_scene_monitor,
#                   get_robot_model, get_trajectory_execution_manager, shutdown
# PlanningComponent methods: plan, set_start_state, set_start_state_to_current_state,
#                            set_goal_state, set_path_constraints, set_workspace, ...
```

`MoveItPy` and `PlanningComponent` have **no `plan_sequence()` method**. Sequence planning MUST go through a raw ROS2 service or action client.

**Two interfaces available** [VERIFIED: official PILZ docs + Python introspection]:
- **Service** `/plan_sequence_path` (`moveit_msgs.srv.GetMotionSequence`) — plans-only, returns trajectories
- **Action** `/sequence_move_group` (`moveit_msgs.action.MoveGroupSequence`) — plan+execute combined

**For look-ahead (plan before execute):** Use the **service** in the background thread. Use `call_async()` from the background thread; the `MultiThreadedExecutor` running in the main thread processes the response and resolves the Future. Poll with `while not future.done(): time.sleep(0.005)` in the background thread (safe — no executor conflict).

**Service name** (authoritative): `/plan_sequence_path` [CITED: moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner]

---

### Finding 2 — MotionSequenceItem and MotionSequenceRequest fields [VERIFIED: Python introspection]

```python
from moveit_msgs.msg import MotionSequenceRequest, MotionSequenceItem, MotionSequenceResponse

# MotionSequenceItem fields:
#   req: moveit_msgs/MotionPlanRequest  (the per-segment planning request)
#   blend_radius: double                ← THE blend_radius field (on the ITEM, not req)

# MotionSequenceRequest fields:
#   items: sequence<moveit_msgs/MotionSequenceItem>

# MotionSequenceResponse fields:
#   error_code: moveit_msgs/MoveItErrorCodes
#   sequence_start: moveit_msgs/RobotState
#   planned_trajectories: sequence<moveit_msgs/RobotTrajectory>  ← N trajectories, one per item
#   planning_time: double
```

**Blend radius field name:** `MotionSequenceItem.blend_radius` (float64) — **NOT** inside `MotionPlanRequest`.

**PILZ constraint** [CITED: moveit.picknik.ai]: Only `items[0].req.start_state` may be populated. Items 1, 2, … inherit start from the previous item's goal. Setting `start_state` on items 1+ is IGNORED.

**Success check:**
```python
from moveit_msgs.msg import MoveItErrorCodes
# MoveItErrorCodes.SUCCESS == 1
if response.error_code.val == MoveItErrorCodes.SUCCESS:
    ...
```

---

### Finding 3 — MotionPlanRequest fields (per segment request) [VERIFIED: Python introspection]

```python
from moveit_msgs.msg import MotionPlanRequest
# Key fields needed for each MotionSequenceItem.req:
#   group_name: str                          (e.g., 'ur_manipulator')
#   pipeline_id: str                         ('pilz_industrial_motion_planner')
#   planner_id: str                          ('LIN', 'PTP', 'CIRC')
#   start_state: moveit_msgs/RobotState      (only on items[0])
#   goal_constraints: list[Constraints]      (MUST be set — see Finding 5)
#   path_constraints: Constraints            (for CIRC paths)
#   num_planning_attempts: int
#   allowed_planning_time: float
#   max_velocity_scaling_factor: float
#   max_acceleration_scaling_factor: float
```

---

### Finding 4 — Setting start_state from a RobotState object [VERIFIED: Python introspection]

There are **two distinct** APIs depending on context:

#### For `MotionSequenceRequest.items[0].req.start_state` (the sequence planner):
Use `moveit_msgs.msg.RobotState` directly (this is a ROS2 message, not a moveit_py object):

```python
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

robot_state_msg = RobotState()
robot_state_msg.joint_state.name = list(joint_names)          # list[str]
robot_state_msg.joint_state.position = list(joint_positions)  # list[float]
robot_state_msg.joint_state.velocity = [0.0] * len(joint_names)
# Assign to first item
items[0].req.start_state = robot_state_msg
```

#### For `PlanningComponent.set_start_state()` (if used — NOT used by sequence planner path):
```python
# Signature (verified):
# set_start_state(configuration_name: str, robot_state: moveit_msgs.msg.RobotState)
planning_component.set_start_state(robot_state=robot_state_msg)
```
`set_start_state()` also takes `moveit_msgs.msg.RobotState`, **not** `moveit.core.robot_state.RobotState`.

#### Getting current robot state as moveit_msgs/RobotState (for first group's start_state):
```python
from moveit.core.robot_state import robotStateToRobotStateMsg

with self._moveit.get_planning_scene_monitor().read_only() as scene:
    current_state = scene.current_state  # moveit.core.robot_state.RobotState

current_state_msg = robotStateToRobotStateMsg(current_state)  # → moveit_msgs/RobotState
```

---

### Finding 5 — How to extract end-state joints from a planned trajectory [VERIFIED: Python introspection + msg inspection]

The `MotionSequenceResponse.planned_trajectories` list contains `moveit_msgs/RobotTrajectory` messages. Each has a `joint_trajectory` (`trajectory_msgs/JointTrajectory`) with `joint_names` and `points[-1]` as the final waypoint.

```python
# Extract end-state for state propagation (D-08):
last_trajectory = response.planned_trajectories[-1]
jt = last_trajectory.joint_trajectory
last_point = jt.points[-1]

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
predicted_end_state = RobotState()
predicted_end_state.joint_state.name = list(jt.joint_names)
predicted_end_state.joint_state.position = list(last_point.positions)
predicted_end_state.joint_state.velocity = [0.0] * len(jt.joint_names)  # stopped at goal
```

This `predicted_end_state` is stored as `last_predicted_state` and assigned to `items[0].req.start_state` for the next group.

---

### Finding 6 — Building goal_constraints from PoseStamped (for MotionPlanRequest) [VERIFIED: official docs patterns]

`moveit.core.kinematic_constraints` Python module does NOT expose `constructGoalConstraints()` [VERIFIED: Python introspection]. It only exposes `construct_link_constraint`, `construct_joint_constraint`, `construct_constraints_from_node`.

Goal constraints must be built manually:

```python
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def build_pose_goal_constraints(
    link_name: str,
    pose_stamped,  # geometry_msgs/PoseStamped
    position_tolerance: float = 0.0001,
    orientation_tolerance: float = 0.001,
) -> Constraints:
    """Build MoveIt2 goal Constraints from a PoseStamped (replaces C++ constructGoalConstraints)."""
    constraints = Constraints()

    # Position constraint
    pos = PositionConstraint()
    pos.header = pose_stamped.header
    pos.link_name = link_name
    pos.weight = 1.0
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
    target = Pose()
    target.position = pose_stamped.pose.position
    target.orientation.w = 1.0
    bv = BoundingVolume()
    bv.primitives = [sphere]
    bv.primitive_poses = [target]
    pos.constraint_region = bv
    constraints.position_constraints = [pos]

    # Orientation constraint
    ori = OrientationConstraint()
    ori.header = pose_stamped.header
    ori.link_name = link_name
    ori.orientation = pose_stamped.pose.orientation
    ori.absolute_x_axis_tolerance = orientation_tolerance
    ori.absolute_y_axis_tolerance = orientation_tolerance
    ori.absolute_z_axis_tolerance = orientation_tolerance
    ori.weight = 1.0
    constraints.orientation_constraints = [ori]

    return constraints
```

This utility should be added to `PilzPlannerService` (private `@staticmethod`) or a `utils/` helper.

---

### Finding 7 — Executing planned trajectories from GetMotionSequence service [VERIFIED: Python introspection]

`MotionSequenceResponse.planned_trajectories` contains N `moveit_msgs/RobotTrajectory` messages (one per item in the request, after blending is applied). For execution:

#### Step 1: Get RobotModel + reference RobotState
```python
robot_model = self._moveit.get_robot_model()
with self._moveit.get_planning_scene_monitor().read_only() as scene:
    ref_state = scene.current_state  # moveit.core.robot_state.RobotState
```

#### Step 2: Convert ROS2 msg → moveit_py RobotTrajectory
```python
from moveit.core.robot_trajectory import RobotTrajectory

# RobotTrajectory.__init__(robot_model)  ← only robot_model, no group_name
traj = RobotTrajectory(robot_model)
traj.set_robot_trajectory_msg(ref_state, ros_traj_msg)
# Args: robot_state (moveit.core.robot_state.RobotState), msg (moveit_msgs.msg.RobotTrajectory)
```

#### Step 3: Execute via TrajectoryExecutionManager
```python
tem = self._moveit.get_trajectory_execution_manager()
for ros_traj_msg in plan_result_dto.trajectories:
    traj = RobotTrajectory(robot_model)
    traj.set_robot_trajectory_msg(ref_state, ros_traj_msg)
    tem.push(traj)
tem.execute_and_wait()
```

**⚠️ Pitfall:** `TrajectoryExecutionManager.execute_and_wait()` vs `MoveItPy.execute()` — see §Common Pitfalls.

---

### Finding 8 — ROS2 ActionServer cancel_callback signature and return type [VERIFIED: Python introspection]

```python
from rclpy.action import ActionServer, CancelResponse

# Default cancel callback signature (verified from rclpy source):
def default_cancel_callback(cancel_request):
    return CancelResponse.REJECT

# Our implementation (D-10):
def _cancel_callback(self, cancel_request) -> CancelResponse:
    """Non-blocking: signal planner to cancel, return immediately."""
    if self._planner_service is not None:
        self._planner_service.cancel()
    return CancelResponse.ACCEPT
```

**ActionServer accepts `cancel_callback` as keyword argument:**
```python
ActionServer(
    self,
    ExecuteTrajectory,
    "movement_controller/execute_trajectory",
    execute_callback=self._execute_callback,
    goal_callback=self._goal_callback,
    cancel_callback=self._cancel_callback,  # ← here
    callback_group=ReentrantCallbackGroup(),
)
```

`CancelResponse.ACCEPT` allows the client's cancel to proceed. The in-flight `MoveGroupSequence` execution is interrupted via the queue drain + `StopIteration` sentinel (D-09). No explicit `moveit.stop()` call is needed per D-10.

For checking cancellation inside `_execute_callback`, use `goal_handle.is_cancel_requested` [VERIFIED: Python introspection]:
```python
if goal_handle.is_cancel_requested:
    goal_handle.canceled()
    return result
```

---

### Finding 9 — Thread-safe queue clearing [VERIFIED: Python runtime test]

For `PilzPlannerService.cancel()` (D-09):

```python
import queue
import threading

# CORRECT — atomic clear under mutex
with self._plan_queue.mutex:
    self._plan_queue.queue.clear()

# Then push sentinel so iterator terminates
self._plan_queue.put(StopIteration)
```

**Do NOT use the drain loop** (`while not q.empty(): q.get_nowait()`) — it has a TOCTOU race: another thread could `put()` between the `empty()` check and `get_nowait()`. The `with q.mutex` approach is the only truly atomic clear.

---

### Finding 10 — move_group capability configuration requirement [VERIFIED: official PILZ docs]

The `/plan_sequence_path` service and `/sequence_move_group` action are **NOT enabled by default** in move_group. They require the following capability to be registered:

```yaml
# In move_group launch / config:
move_group_capabilities:
  capabilities: >
    pilz_industrial_motion_planner/MoveGroupSequenceAction
    pilz_industrial_motion_planner/MoveGroupSequenceService
```

[CITED: moveit.picknik.ai — "For this service and action, the move_group launch file needs to be modified to include these Pilz Motion Planner capabilities."]

**Impact:** The `ur_moveit_config` launch file used in the project must either already include these capabilities or must be patched. This is a deployment prerequisite the planner must include as a task.

---

## Standard Stack

### Core (Phase 4 additions — no new pip packages)
| Import | Purpose |
|--------|---------|
| `moveit_msgs.srv.GetMotionSequence` | Service type for sequence planning |
| `moveit_msgs.msg.MotionSequenceRequest`, `MotionSequenceItem` | Build per-group planning requests |
| `moveit_msgs.msg.MotionSequenceResponse` | Parse planning result |
| `moveit_msgs.msg.MotionPlanRequest` | Per-segment request embedded in `MotionSequenceItem.req` |
| `moveit_msgs.msg.RobotState` | start_state for first item; end-state propagation |
| `moveit_msgs.msg.MoveItErrorCodes` | Success check (`val == 1`) |
| `moveit.core.robot_trajectory.RobotTrajectory` | Convert/wrap trajectories for execution |
| `moveit.core.robot_state.robotStateToRobotStateMsg` | Convert `RobotState` → `moveit_msgs/RobotState` |
| `rclpy.action.CancelResponse` | Return type of `cancel_callback` |
| `threading.Thread`, `threading.Event` | Background look-ahead thread + cancel flag |
| `queue.Queue` | Thread-safe trajectory result queue |

All are **already available** in the devcontainer (`ros:jazzy-ros-base` + installed ROS2 packages). No `pip install` required.

---

## Package Legitimacy Audit

> No new external packages are installed in Phase 4. All dependencies are ROS2 Jazzy built-in packages already present in the devcontainer. This section is not applicable.

---

## Architecture Patterns

### System Architecture Diagram

```
                    ┌─────────────────────────────────────────────────────┐
                    │  URMovementController._execute_callback (async)      │
                    │                                                      │
  Goal received ──► │  1. Parse TrajectoryGoalDTO                         │
                    │  2. TrajectoryGrouper.group(paths) → groups[]        │
                    │  3. planner_service.plan_all(groups)  ─────────────────► background thread starts
                    │  4. for dto in planner_service.iterate_planned():    │
                    │       a. publish 'executing' feedback (group-level)   │
                    │       b. execute pre-planned trajectory segments      │◄── dequeue from plan_queue
                    │       c. publish 'completed' feedback                 │
                    │  5. cancel check via goal_handle.is_cancel_requested  │
                    └─────────────────────────────────────────────────────┘
                               ▲ blocking dequeue
                               │
                    ┌──────────┴──────────────────────────────────────────┐
                    │  PilzPlannerService background thread (plan_all)     │
                    │                                                      │
                    │  for group in groups:                                │
                    │    1. Build MotionSequenceRequest                    │
                    │       items[0].req.start_state = last_predicted_state │
                    │       items[i].blend_radius = path_dto.blend_radius   │
                    │       items[i].req.goal_constraints = build_pose(...)  │
                    │    2. call GetMotionSequence service (call_async)     │──►  /plan_sequence_path
                    │    3. poll future until done (cancel-aware)           │◄──  PILZ sequence planner
                    │    4. extract end state → last_predicted_state        │
                    │    5. plan_queue.put(PlanResultDTO(...))              │
                    │  plan_queue.put(StopIteration)                        │
                    └──────────────────────────────────────────────────────┘
```

### Recommended Project Structure Changes

```
src/movement_controller/movement_controller/
├── services/
│   └── pilz_planner_service.py    ← EXPAND (plan_all, iterate, cancel, _plan_group_sequence)
├── models/
│   └── plan_result_dto.py         ← EXPAND (add path_ids: list[str], blended: bool, trajectories: list)
├── ur_movement_controller.py      ← UPDATE (_execute_callback: generator loop, cancel_callback)
└── utils/
    └── motion_sequence_builder.py ← NEW (build_pose_goal_constraints helper; optional but clean)
```

---

## Architecture Pattern 1 — GetMotionSequence Service from Background Thread

```python
# Source: verified Python introspection + official PILZ docs

import time
import threading
import queue
from moveit_msgs.srv import GetMotionSequence
from moveit_msgs.msg import (
    MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest,
    MoveItErrorCodes, RobotState,
)
from moveit.core.robot_state import robotStateToRobotStateMsg

class PilzPlannerService:
    def __init__(self, moveit, moveit_group_name: str, node) -> None:
        self._moveit = moveit
        self._group_name = moveit_group_name
        self._node = node
        self._planning_component = moveit.get_planning_component(moveit_group_name)
        # Service client for sequence planning
        self._plan_seq_client = node.create_client(GetMotionSequence, '/plan_sequence_path')

    def plan_all(self, groups: list) -> None:
        """Start fresh background planning thread + queue (D-04)."""
        self._cancel_event = threading.Event()
        self._plan_queue = queue.Queue()
        self._planning_thread = threading.Thread(
            target=self._planning_loop, args=(groups,), daemon=True
        )
        self._planning_thread.start()

    def _planning_loop(self, groups: list) -> None:
        """Background thread: plan each group sequentially (D-08)."""
        last_predicted_state: RobotState | None = None

        # Get current state for first group's start_state
        with self._moveit.get_planning_scene_monitor().read_only() as scene:
            current_moveit_state = scene.current_state
        last_predicted_state = robotStateToRobotStateMsg(current_moveit_state)

        for group in groups:
            if self._cancel_event.is_set():
                break

            result = self._plan_group_sequence(group, last_predicted_state)

            if self._cancel_event.is_set():
                break

            if result is None:
                # Planning failed — push error sentinel (StopIteration terminates iterator)
                self._plan_queue.put(StopIteration)
                return

            # Extract end state for next group (D-08)
            last_predicted_state = self._extract_end_state(result.trajectories)
            self._plan_queue.put(result)

        self._plan_queue.put(StopIteration)

    def _plan_group_sequence(self, group, start_state_msg: RobotState):
        """Call /plan_sequence_path service from background thread."""
        request = GetMotionSequence.Request()
        seq_req = MotionSequenceRequest()

        for i, path_dto in enumerate(group):
            item = MotionSequenceItem()
            item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
            item.req.group_name = self._group_name
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', 'CIRC'
            item.req.allowed_planning_time = 5.0
            item.req.max_velocity_scaling_factor = 0.1
            item.req.max_acceleration_scaling_factor = 0.1
            item.req.goal_constraints = [
                build_pose_goal_constraints(
                    path_dto.tool_frame or 'tool0', path_dto.target_pose
                )
            ]
            if i == 0:
                item.req.start_state = start_state_msg
            if path_dto.motion_type.value == 'CIRC':
                item.req.path_constraints = self._build_circ_constraints(path_dto)
            seq_req.items.append(item)

        request.request = seq_req
        future = self._plan_seq_client.call_async(request)

        # Poll until done — executor in main thread processes the response (thread-safe)
        while not future.done():
            if self._cancel_event.is_set():
                return None
            time.sleep(0.005)

        response = future.result()
        if response is None or response.response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        return PlanResultDTO(
            success=True,
            trajectories=list(response.response.planned_trajectories),
            path_ids=[p.path_id for p in group],
            blended=len(group) > 1,
        )
```

---

## Architecture Pattern 2 — generator + queue consumption (D-05)

```python
def iterate_planned_trajectories(self):
    """Generator: yields PlanResultDTO from queue; terminates on StopIteration sentinel."""
    while True:
        item = self._plan_queue.get()  # blocks if empty
        if item is StopIteration:
            return
        yield item
```

---

## Architecture Pattern 3 — Execute planned segments (controller side)

```python
# In URMovementController._execute_callback:
from moveit.core.robot_trajectory import RobotTrajectory

robot_model = self._moveit.get_robot_model()
tem = self._moveit.get_trajectory_execution_manager()

self._planner_service.plan_all(groups)

for plan_dto in self._planner_service.iterate_planned_trajectories():
    if goal_handle.is_cancel_requested:
        self._planner_service.cancel()
        goal_handle.canceled()
        return result

    # Publish group-level 'executing' feedback (D-01)
    fb = ExecuteTrajectory.Feedback()
    fb.status = FeedbackStatusEnum.EXECUTING.value
    fb.trajectory_path_ids = plan_dto.path_ids
    goal_handle.publish_feedback(fb)

    # Execute all trajectory segments for this group
    with self._moveit.get_planning_scene_monitor().read_only() as scene:
        ref_state = scene.current_state

    for ros_traj_msg in plan_dto.trajectories:
        traj = RobotTrajectory(robot_model)
        traj.set_robot_trajectory_msg(ref_state, ros_traj_msg)
        tem.push(traj)
    tem.execute_and_wait()

    completed_ids.extend(plan_dto.path_ids)

    # Publish group-level 'completed' feedback (D-01)
    fb.status = FeedbackStatusEnum.COMPLETED.value
    goal_handle.publish_feedback(fb)
```

---

## Architecture Pattern 4 — cancel() implementation (D-09)

```python
def cancel(self) -> None:
    """Thread-safe cancellation (D-09)."""
    self._cancel_event.set()
    # Atomically drain the queue (thread-safe — uses Queue internal mutex)
    with self._plan_queue.mutex:
        self._plan_queue.queue.clear()
    # Push sentinel so iterate_planned_trajectories() terminates
    self._plan_queue.put(StopIteration)
```

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Blend trajectory planning | Custom blending code | `MotionSequenceRequest` + `/plan_sequence_path` service | PILZ handles blend radius math, velocity continuity, overlap checks |
| Goal constraints from Pose | Custom IK or manual FK | `build_pose_goal_constraints()` utility (see Finding 6) | Correct tolerance ball + quaternion constraint |
| Thread-safe queue drain | Loop with `empty()`+`get_nowait()` | `with queue.mutex: queue.queue.clear()` | Eliminates TOCTOU race |
| Cancel waiting on thread join | `thread.join(timeout=...)` | Set event + drain + push sentinel | D-10 requires non-blocking cancel |
| Sequence execution management | Custom controller streaming | `TrajectoryExecutionManager.push()` + `execute_and_wait()` | Handles controller selection, timeouts |

---

## Common Pitfalls

### Pitfall 1: Last item's blend_radius MUST be 0
**What goes wrong:** PILZ raises `LastBlendRadiusNotZeroException` if `items[-1].blend_radius != 0`.
**Why it happens:** The blend sphere on the last point has no "next goal" to blend into.
**How to avoid:** Always set `items[-1].blend_radius = 0.0` regardless of what the goal says. The `TrajectoryGrouper` algorithm guarantees the LAST path in each group starts a new group OR is a singleton, but within a group, the LAST path's blend_radius from the goal must be overridden to 0.

```python
item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
```

### Pitfall 2: blend_radius overlap constraint
**What goes wrong:** Planning fails with `NegativeBlendRadiusException` or invalid plan if adjacent blend spheres overlap.
**Why it happens:** PILZ requires `blend_radius[i] + blend_radius[i+1] < distance_between_goals`.
**How to avoid:** Validate/warn at goal validation time. In tests, keep blend_radius small (e.g., 0.01m) relative to target distances.

### Pitfall 3: MoveGroupSequenceService capability not loaded in move_group
**What goes wrong:** `/plan_sequence_path` service is unavailable; service client times out waiting.
**Why it happens:** Default `ur_moveit_config` launch does NOT include PILZ sequence capabilities.
**How to avoid:** Add the capability in the move_group config or launch file:
```yaml
move_group_capabilities:
  capabilities: "pilz_industrial_motion_planner/MoveGroupSequenceService"
```

### Pitfall 4: Service client created before service is available
**What goes wrong:** Background thread calls `call_async()` and gets no response.
**How to avoid:** In `on_configure`, wait for service availability with timeout:
```python
if not self._plan_seq_client.wait_for_service(timeout_sec=timeout):
    self.get_logger().error('/plan_sequence_path not available')
    return TransitionCallbackReturn.FAILURE
```

### Pitfall 5: Stale cancel_event / queue from previous goal
**What goes wrong:** A new goal starts with the previous goal's `cancel_event` already set; background thread immediately exits.
**How to avoid:** D-04 mandates creating a **fresh** `queue.Queue()` and `threading.Event()` on each `plan_all()` call. Do NOT reuse.

### Pitfall 6: PilzPlannerService constructor change
**What goes wrong:** `URMovementController` creates `PilzPlannerService` without the new `node` argument — `TypeError`.
**How to avoid:** Update the constructor call in `on_configure`:
```python
# Was:  PilzPlannerService(self._moveit, moveit_group_name)
# Now:  PilzPlannerService(self._moveit, moveit_group_name, node=self)
self._planner_service = PilzPlannerService(self._moveit, moveit_group_name, node=self)
```

### Pitfall 7: Blocking `get_planning_scene_monitor().read_only()` in background thread
**What goes wrong:** The planning scene monitor context manager may block if the main thread holds a write lock.
**How to avoid:** Only call `read_only()` once at the START of the planning thread to get `current_state` for the first group's start_state. Use the service's `response.planned_trajectories[-1].joint_trajectory` for subsequent state propagation — no scene monitor access needed per-group.

### Pitfall 8: TrajectoryExecutionManager multi-push not guaranteed to execute without stops
**What goes wrong:** Pushing N trajectories and calling `execute_and_wait()` may have brief stops between segments with some robot controllers.
**Why it happens:** Standard `follow_joint_trajectory` controllers execute one trajectory to completion, then start the next. Even if the PILZ plan has non-zero boundary velocities, the controller may stop between calls.
**How to avoid:** In simulation with `fake_hardware_interface`, this is acceptable — the "seamless blend" guarantee is the PILZ sequence planner's mathematical output; physical blending depends on the controller. For integration tests, test that trajectories are pre-planned (look-ahead) and that the trajectory PLAN contains blend continuity (non-zero boundary velocities), not that physical motion is seamless. Physical blending requires real hardware with joint-trajectory blending controller support.
**Fallback:** If blending tests fail, fall back to using the `MoveGroupSequence` action (plan+execute per group, no look-ahead) and document as a known limitation.

---

## PlanResultDTO Changes (D-06)

```python
# models/plan_result_dto.py — additions (backward-compatible defaults)
class PlanResultDTO(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    success: bool = Field(description='True if PILZ planning succeeded')
    trajectory: Any = Field(default=None, description='DEPRECATED (Phase 3 single path). None in Phase 4.')
    trajectories: list[Any] = Field(
        default_factory=list,
        description='List of moveit_msgs/RobotTrajectory for each segment in this group',
    )
    error_message: str = Field(default='', description='Human-readable error; empty on success')
    # NEW in Phase 4 (D-06):
    path_ids: list[str] = Field(
        default_factory=list,
        description='Path IDs this result covers, in order',
    )
    blended: bool = Field(
        default=False,
        description='True if this group has more than one path (blend group)',
    )
```

---

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| `moveit_msgs.srv.GetMotionSequence` | Sequence planning | ✓ | Jazzy | — |
| `moveit_msgs.action.MoveGroupSequence` | Fallback execution | ✓ | Jazzy | — |
| `moveit_msgs.msg.MotionSequenceItem` | Per-segment request | ✓ | Jazzy | — |
| `/plan_sequence_path` service | Planning in background thread | ✗ (config-gated) | — | Must enable in move_group capabilities |
| `TrajectoryExecutionManager.push()` | Blended execution | ✓ | Jazzy | — |
| `rclpy.action.CancelResponse` | cancel_callback | ✓ | Jazzy | — |
| `threading.Event`, `queue.Queue` | Look-ahead + cancel | ✓ | Python 3.11+ stdlib | — |

**Blocking prerequisite:** `/plan_sequence_path` service requires `pilz_industrial_motion_planner/MoveGroupSequenceService` capability in `move_group`. The planner MUST add a task to verify or patch the launch/config. This may be already present in the `ur_moveit_config` used in simulation — the integration test should check with `wait_for_service(timeout_sec=5.0)`.

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | `pytest` + `ament_pytest` |
| Config file | `CMakeLists.txt` → `ament_add_pytest_test()` |
| Quick run | `python -m pytest src/movement_controller/tests/unit/ -v` |
| Full suite | `colcon test --packages-select movement_controller && colcon test-result --verbose` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | Notes |
|--------|----------|-----------|-------------------|-------|
| MOT-02 | `MotionSequenceRequest` built for all groups | Unit | `pytest tests/unit/test_pilz_planner_service.py -k sequence_request` | Mock service client |
| MOT-02 | `blend_radius` from goal propagated to `MotionSequenceItem.blend_radius` | Unit | `pytest tests/unit/test_pilz_planner_service.py -k blend_radius` | Assert item.blend_radius == path.blend_radius |
| MOT-02 | Last item's blend_radius is always 0 | Unit | `pytest tests/unit/test_pilz_planner_service.py -k last_item_zero_blend` | — |
| MOT-03 | Look-ahead: group 1 planned before group 0 execution completes | Integration | `pytest tests/integration/test_look_ahead.py -k look_ahead_used` | Inject planning hook to check timing |
| MOT-04 | Generator yields immediately when queue pre-populated | Unit | `pytest tests/unit/test_pilz_planner_service.py -k queue_preloaded` | Put items in queue, assert yield is instant |
| MOT-04 | No re-plan latency: `execute_and_wait` called immediately after dequeue | Integration | `pytest tests/integration/test_look_ahead.py -k no_replan` | Log timestamps |
| D-06 | `PlanResultDTO.path_ids` contains correct IDs | Unit | `pytest tests/unit/test_plan_result_dto.py -k path_ids` | — |
| D-06 | `PlanResultDTO.blended` is True for 2+ path group | Unit | `pytest tests/unit/test_plan_result_dto.py -k blended_flag` | — |
| D-09 | `cancel()` drains queue and pushes sentinel | Unit | `pytest tests/unit/test_pilz_planner_service.py -k cancel_drains_queue` | — |
| D-09 | Generator terminates after cancel() | Unit | `pytest tests/unit/test_pilz_planner_service.py -k generator_terminates` | — |
| D-10 | `cancel_callback` returns in < 10ms | Unit | `pytest tests/unit/test_ur_movement_controller.py -k cancel_callback_nonblocking` | Measure elapsed time |

### Wave 0 Gaps (new test files needed)
- [ ] `tests/unit/test_pilz_planner_service.py` — covers MOT-02, MOT-03, MOT-04, D-09 (mock `GetMotionSequence` service client)
- [ ] `tests/unit/test_plan_result_dto.py` — covers D-06 field validation
- [ ] `tests/integration/test_look_ahead.py` — covers MOT-03, MOT-04 with mocked MoveItPy + fake service client

### Mocking Strategy for Unit Tests
```python
# Mock the GetMotionSequence service client
@pytest.fixture
def mock_plan_seq_client(mocker):
    client = mocker.MagicMock()
    future = mocker.MagicMock()
    future.done.return_value = True
    # Build a fake MotionSequenceResponse
    from moveit_msgs.msg import MotionSequenceResponse, MoveItErrorCodes
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from moveit_msgs.msg import RobotTrajectory
    resp = MotionSequenceResponse()
    resp.error_code.val = MoveItErrorCodes.SUCCESS
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = ['joint1', 'joint2']
    point = JointTrajectoryPoint()
    point.positions = [0.1, 0.2]
    point.velocities = [0.0, 0.0]
    traj.joint_trajectory.points = [point]
    resp.planned_trajectories = [traj]
    future.result.return_value = mocker.MagicMock(response=resp)
    client.call_async.return_value = future
    return client
```

---

## Security Domain

> Phase 4 is internal ROS2 node implementation. No external boundaries (no user input, no network endpoints). OWASP/ASVS categories V2/V3/V4/V6 are not applicable. V5 Input Validation applies only to goal message validation (inherited from Phase 2/3 `TrajectoryGoalDTO.from_ros_msg()`).

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V5 Input Validation | Yes (inherited) | `TrajectoryGoalDTO.from_ros_msg()` with Pydantic v2 |
| V2-V4 Auth/Session/Access | No | Internal ROS2 node |
| V6 Cryptography | No | No crypto |

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `TrajectoryExecutionManager.push()` + `execute_and_wait()` executes N blended segments continuously (no inter-segment stop) | Finding 7 / Pitfall 8 | Blended execution shows stops between segments; must fall back to `MoveGroupSequence` action |
| A2 | `call_async()` from background thread with MultiThreadedExecutor polling is thread-safe | Pattern 1 | Background thread future never resolves → hang; need timeout/cancel logic |
| A3 | `ur_moveit_config` used in simulation does NOT already include `MoveGroupSequenceService` capability | Pitfall 3 | If already included, launch file patch task is unnecessary |

---

## Open Questions

1. **Does `TrajectoryExecutionManager.push()` + `execute_and_wait()` produce seamless blended motion with `fake_hardware_interface`?**
   - What we know: PILZ blended trajectories have non-zero velocity at segment boundaries
   - What's unclear: Whether `TrajectoryExecutionManager` streams segments as continuous motion or inserts stops
   - Recommendation: Add an integration test that checks trajectory boundary velocities; if stops occur, use `MoveGroupSequence` action as fallback

2. **Is `ur_moveit_config` for Jazzy already configured with PILZ sequence capabilities?**
   - What we know: The service capability must be explicitly added
   - Recommendation: Integration test should include `wait_for_service('/plan_sequence_path', timeout_sec=5.0)` to fail fast

3. **Does `PlanningSceneMonitor.read_only()` block in the background thread?**
   - What we know: It acquires a read lock
   - Recommendation: Only call it ONCE at the start of `_planning_loop`, cache the state

---

## Sources

### Primary (HIGH confidence)
- Python runtime introspection of `/opt/ros/jazzy/lib/python3.12/site-packages/moveit/` — all API findings
- Official PILZ docs: `https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html`
- `moveit_msgs` Python package introspection — message field names, types

### Secondary (MEDIUM confidence)
- `pilz_sequence.cpp` tutorial from `github.com/moveit/moveit2_tutorials` — action/service usage patterns

### Tertiary (LOW confidence — verified otherwise)
- rclpy internals for `call_async()` + executor threading behavior (A2 above)

---

## Metadata

**Confidence breakdown:**
- API findings (sequence request, blend_radius, cancel_callback): HIGH — verified by Python introspection
- Architecture (GetMotionSequence service as planning path): HIGH — aligns with PILZ official docs
- Execution (TrajectoryExecutionManager for blended segments): MEDIUM — theoretical; Pitfall 8 flags the uncertainty
- Thread safety of `call_async()` from background thread: MEDIUM — standard rclpy pattern, but not tested in this environment

**Research date:** 2026-05-29
**Valid until:** 2026-07-01 (ROS2 Jazzy stable; moveit_py API unlikely to change in 30 days)

---

## RESEARCH COMPLETE

**Phase:** 4 — Look-Ahead Planning & Blended Multi-Path Execution
**Confidence:** HIGH (core APIs), MEDIUM (execution blending)

### Key Findings
- `moveit_py` has NO `plan_sequence()` — must use raw ROS2 service `/plan_sequence_path` (`GetMotionSequence`)
- `MotionSequenceItem.blend_radius` (float64) is the blend field — NOT inside `MotionPlanRequest`
- `PilzPlannerService` constructor must gain a `node` parameter to create the service client
- `PlanningComponent.set_start_state()` and `MotionPlanRequest.start_state` both accept `moveit_msgs.msg.RobotState` (the ROS2 message), NOT `moveit.core.robot_state.RobotState`
- Thread-safe queue drain: `with q.mutex: q.queue.clear()` (not drain loop)
- `cancel_callback(self, cancel_request) -> CancelResponse` — returns `CancelResponse.ACCEPT`
- `/plan_sequence_path` service requires `pilz_industrial_motion_planner/MoveGroupSequenceService` capability in move_group — this must be verified/patched in the launch config

### File Created
`.planning/phases/04-look-ahead-planning-and-blended-multi-path-execution/04-RESEARCH.md`

### Confidence Assessment
| Area | Level | Reason |
|------|-------|--------|
| Standard Stack | HIGH | All imports verified by Python introspection |
| MotionSequenceRequest API | HIGH | Verified: fields, blend_radius location, PILZ constraints |
| State propagation approach | HIGH | Verified: message types, joint extraction |
| cancel_callback signature | HIGH | Verified: Python introspection of rclpy source |
| Thread-safe queue clearing | HIGH | Verified: runtime test |
| TrajectoryExecutionManager blending | MEDIUM | API verified; physical behavior with UR controller uncertain |
| move_group capability requirement | HIGH | Official PILZ docs explicit |

### Open Questions
- Does `TrajectoryExecutionManager.push()` + `execute_and_wait()` produce seamless motion or inter-segment stops?
- Is `ur_moveit_config` already configured with PILZ sequence capabilities?

### Ready for Planning
Research complete. Planner can now create PLAN.md files.
