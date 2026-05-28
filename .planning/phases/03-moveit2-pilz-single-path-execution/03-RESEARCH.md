# Phase 3: MoveIt2 + PILZ Single-Path Execution — Research

**Researched:** 2026-05-28
**Domain:** MoveIt2 Python API (`moveit_py`) + PILZ Industrial Motion Planner
**Confidence:** HIGH (API signatures verified against moveit2 GitHub source and official docs)

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01:** `MoveItPy` instantiated in `on_configure` by `URMovementController`; controller owns the lifecycle.
- **D-02:** `on_configure` creates `MoveItPy`, calls `get_planning_component('ur_manipulator')`, injects component into `PilzPlannerService`.
- **D-03:** `moveit_connection_timeout` (float, default 10.0 s) declared as node parameter in `on_configure`.
- **D-04:** Connection failure → log ERROR, return `TransitionCallbackReturn.FAILURE`; node stays UNCONFIGURED.
- **D-05:** `on_cleanup` destroys `MoveItPy`, sets reference to `None`; `on_configure` re-creates.
- **D-06:** `PilzPlannerService` is a plain Python class in `services/pilz_planner_service.py`; receives planning component via constructor; NOT a ROS2 node.
- **D-07:** `plan()` maps `MotionTypeEnum` → `pipeline_id='pilz_industrial_motion_planner'`, `planner_id='LIN'|'PTP'|'CIRC'`.
- **D-08:** Service calls `set_start_state_to_current_state()` at the start of every `plan()` call.
- **D-09:** Controller holds `self._moveit` (MoveItPy) + `self._planner_service` (PilzPlannerService). After successful plan, calls `self._moveit.execute(trajectory, blocking=True, controllers=[])`.
- **D-10:** CIRC modes: `circ_type == 'interim'` → circ_point is the arc waypoint; `circ_type == 'center'` → circ_point is arc center.
- **D-11:** CIRC path with empty/invalid `circ_type` is rejected in `goal_callback` with a clear error.
- **D-12:** CIRC validation added to `goal_callback` structural validation block.
- **D-13:** `execute_callback` flattens multi-path groups to individual paths; each path planned+executed separately in Phase 3.
- **D-14:** Phase 4 will replace flatten logic with `MoveGroupSequence`. Phase 3 produces stop-start execution intentionally.
- **D-15:** Feedback per path: `{status: 'executing', path_ids}` → `{status: 'completed', path_ids}`.
- **D-16:** Fail-fast on first planning failure; goal aborted immediately.
- **D-17:** Execution failure also aborts immediately; `error_message` identifies the failing `path_id`.
- **D-18:** Errors logged at ERROR level before returning failure result.
- **D-19:** Simulation smoke test mocks `MoveItPy` at Python level; no real `move_group` node required.

### Agent's Discretion

- `PlanResult` internal type: dataclass vs Pydantic model (project says Pydantic for DTOs crossing boundaries; PlanResult stays within the package — either is acceptable; Pydantic with `arbitrary_types_allowed=True` is the safer choice for consistency).
- Velocity/acceleration scaling from `TrajectoryPathDTO.cartesian_speed` / `acceleration` → must map to `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` (floats 0–1). Since CON-01–CON-06 are Phase 5, use a safe default or pass through directly if ≤ 1.0.
- CIRC validation for `goal_callback` can be inline in controller OR delegated to extended `TrajectoryPathDTO.from_ros_msg` validator — agent/planner decides based on API findings (both are acceptable per D-12).

### Deferred Ideas (OUT OF SCOPE)

- `MoveGroupSequence` blended execution — Phase 4.
- Look-ahead parallel planning — Phase 4.
- Scene management — Phase 6.
- Motion constraints (CON-01–CON-06) — Phase 5.
- Launch files — Phase 7.
- Action server cancellation — Phase 3/4 boundary.
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| MOT-01 | PILZ Industrial Motion Planner plugin used for planning (LIN, PTP, CIRC pipeline IDs) | §Standard Stack: pipeline name `pilz_industrial_motion_planner`; planner IDs `LIN`, `PTP`, `CIRC` verified |
| MOT-02 | Multi-path trajectories with blending via `MoveGroupSequence` | Phase 3 uses single-path only; blending is Phase 4. Phase 3 proves PILZ works first. |
| MOT-05 | Action server rejects new goals while trajectory executes | Already implemented in Phase 2 (`_is_executing` flag + lock); Phase 3 preserves this. |
</phase_requirements>

---

## Summary

Phase 3 wires `MoveItPy` into the existing `URMovementController` LifecycleNode and implements `PilzPlannerService`. The result is a node that can plan LIN, PTP, and CIRC paths using the PILZ planner and execute them against the robot, with full feedback delivery.

The `moveit_py` Python bindings expose `MoveItPy`, `PlanningComponent`, and `PlanRequestParameters`. Planning pipeline and planner are set via `PlanRequestParameters` attributes (`planning_pipeline`, `planner_id`) — NOT via method calls on `PlanningComponent`. CIRC support requires setting `path_constraints` on the planning component with a `moveit_msgs.msg.Constraints` message whose `name` field selects `'interim'` or `'center'` mode.

`ros-jazzy-moveit` is **not installed** in the current devcontainer. Plan 1 must add it to the Dockerfile before any other work can proceed.

**Primary recommendation:** Install `ros-jazzy-moveit` (adds `moveit_py`), use `PlanRequestParameters` with direct attribute assignment for PILZ pipeline selection, use `set_path_constraints` for CIRC, mock `MoveItPy` at the module level for tests.

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|-----------|-------------|----------------|-----------|
| MoveItPy lifecycle mgmt | `URMovementController` (LifecycleNode) | — | Controller owns connection; MoveItPy is created/destroyed with configure/cleanup |
| PILZ pipeline selection | `PilzPlannerService` (plain Python) | — | Encapsulates motion type → planner_id mapping; controller stays generic |
| Goal validation (CIRC) | `URMovementController._goal_callback` | `TrajectoryPathDTO` validators | Structural rejection before accept; consistent with Phase 2 D-03 pattern |
| Trajectory execution | `URMovementController._execute_callback` | `MoveItPy.execute()` | Controller orchestrates; MoveItPy handles low-level controller dispatch |
| Feedback publication | `URMovementController._execute_callback` | — | Same feedback contract established in Phase 2 |

---

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `moveit_py` | ships with `ros-jazzy-moveit` | MoveIt2 Python bindings for planning + execution | Only supported MoveIt2 Python API in ROS2 |
| `pilz_industrial_motion_planner` | ships with `ros-jazzy-moveit` | PILZ plugin for deterministic LIN/PTP/CIRC motion | Locked decision; deterministic vs OMPL stochastic |
| `moveit_msgs` | ships with `ros-jazzy-moveit` | `Constraints`, `PositionConstraint` for CIRC path | Standard MoveIt2 message types |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| `shape_msgs.msg.SolidPrimitive` | ROS2 Jazzy | Bounding volume for CIRC constraint region | CIRC path planning via `path_constraints` |
| `geometry_msgs.msg.Pose` | ROS2 Jazzy | Pose inside bounding volume primitive | Locating circ_point in CIRC constraint |

### Installation
```bash
# Add to Dockerfile (before venv setup, after base apt update):
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*
```

**Verified availability:** `rosdep resolve moveit_py` → `ros-jazzy-moveit-py` (the Python bindings are included in `ros-jazzy-moveit`). The ROS2 apt source at `packages.ros.org/ros2/ubuntu` is already configured in `/etc/apt/sources.list.d/ros2.sources` — `apt-get update` is required first. [VERIFIED: rosdep resolution in devcontainer]

---

## Package Legitimacy Audit

> These are ROS2 ecosystem packages installed via apt from the official ROS2 package repository, not PyPI or npm. The slopcheck tool is for PyPI/npm packages. All packages below ship with the official `ros-jazzy-moveit` apt meta-package.

| Package | Registry | Age | Source Repo | Disposition |
|---------|----------|-----|-------------|-------------|
| `ros-jazzy-moveit` | ROS2 apt | 10+ yrs | github.com/moveit/moveit2 | Approved |
| `ros-jazzy-moveit-py` | ROS2 apt | ~3 yrs | github.com/moveit/moveit2 | Approved |
| `ros-jazzy-pilz-industrial-motion-planner` (bundled in moveit) | ROS2 apt | ~5 yrs | github.com/moveit/moveit2 | Approved |

**Packages removed due to slopcheck [SLOP] verdict:** none

---

## Architecture Patterns

### System Architecture Diagram

```
[ExecuteTrajectory action goal]
        │
        ▼
[URMovementController._goal_callback]
  ├── lifecycle active check
  ├── _is_executing check + acquire flag
  └── structural validation (DTO + CIRC type)
        │ ACCEPT
        ▼
[URMovementController._execute_callback]
  ├── TrajectoryGoalDTO.from_ros_msg(goal)
  ├── TrajectoryGrouper.group(paths) → groups
  └── for each path in flattened groups:
        ├── publish feedback {executing}
        ├── PilzPlannerService.plan(path_dto) → PlanResult
        │     ├── set_start_state_to_current_state()
        │     ├── set_goal_state(pose_stamped, pose_link)
        │     ├── [CIRC only] set_path_constraints(Constraints)
        │     ├── PlanRequestParameters(pipeline, planner_id)
        │     └── planning_component.plan(single_plan_parameters)
        │           └── [FAIL] → PlanResult(success=False) → abort goal
        ├── [plan success] moveit.execute(trajectory, controllers=[])
        │     └── ExecutionStatus → [FAIL] → abort goal
        └── publish feedback {completed}
              │ after all paths
              ▼
        [return Result(success=True)]
```

### Recommended Project Structure (Phase 3 additions)

```
movement_controller/
    ur_movement_controller.py          # MODIFY: add MoveItPy + PilzPlannerService wiring
    services/
        __init__.py                    # exists (empty stub)
        pilz_planner_service.py        # NEW: PilzPlannerService class
    models/
        plan_result_dto.py             # NEW: PlanResultDTO (internal, planning success/failure)
tests/
    unit/
        test_pilz_planner_service.py   # NEW: unit tests for PilzPlannerService
        test_ur_movement_controller.py # EXTEND: add MoveItPy integration tests
    integration/
        test_execute_trajectory_smoke.py  # NEW: full smoke test with mocked MoveItPy
```

### Pattern 1: PlanRequestParameters for PILZ (VERIFIED)

```python
# Source: github.com/moveit/moveit2/moveit_py/moveit/planning.pyi + planning_component.cpp
from moveit.planning import MoveItPy, PlanRequestParameters

# In PilzPlannerService.plan():
params = PlanRequestParameters(self._moveit_instance, '')  # '' = no ROS param namespace
params.planner_id = 'LIN'          # or 'PTP' or 'CIRC'
params.planning_pipeline = 'pilz_industrial_motion_planner'
params.planning_attempts = 1
params.planning_time = 5.0
params.max_velocity_scaling_factor = 0.1   # see notes on speed mapping below
params.max_acceleration_scaling_factor = 0.1

plan_result = self._planning_component.plan(single_plan_parameters=params)
```

**Important:** `PlanRequestParameters` takes the `MoveItPy` instance as first arg (it loads default values from the node's ROS parameters). Attributes are then overridable directly. [VERIFIED: moveit2/moveit_py/src/moveit/moveit_ros/moveit_cpp/planning_component.cpp L198-L235]

### Pattern 2: Goal State and Start State (VERIFIED)

```python
# Source: MoveIt2 Python API tutorial + planning_component.cpp bindings
from geometry_msgs.msg import PoseStamped

# Reset start state every plan call
self._planning_component.set_start_state_to_current_state()

# Set pose goal
pose_goal = path_dto.target_pose  # already a PoseStamped
self._planning_component.set_goal_state(
    pose_stamped_msg=pose_goal,
    pose_link=path_dto.tool_frame or 'tool0'  # tool frame from DTO, fallback to tool0
)
```

[VERIFIED: moveit2/moveit_py/src/moveit/moveit_ros/moveit_cpp/planning_component.cpp L305-L335]

### Pattern 3: CIRC Path Constraints (VERIFIED from PILZ docs + moveit_py bindings)

```python
# Source: https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def _build_circ_constraints(path_dto: TrajectoryPathDTO) -> Constraints:
    """Build path constraints for PILZ CIRC planner."""
    constraints = Constraints()
    # name 'interim' or 'center' — PILZ reads this to determine the mode
    constraints.name = path_dto.circ_type.value  # 'interim' or 'center'

    pos_constraint = PositionConstraint()
    pos_constraint.header.frame_id = path_dto.target_pose.header.frame_id
    pos_constraint.link_name = path_dto.tool_frame or 'tool0'

    # Bounding volume: tiny sphere located AT the circ_point
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.001]  # radius ~0; just a position marker

    point_pose = Pose()
    point_pose.position.x = path_dto.circ_point.x
    point_pose.position.y = path_dto.circ_point.y
    point_pose.position.z = path_dto.circ_point.z
    point_pose.orientation.w = 1.0

    bv = BoundingVolume()
    bv.primitives = [sphere]
    bv.primitive_poses = [point_pose]
    pos_constraint.constraint_region = bv

    constraints.position_constraints = [pos_constraint]
    return constraints

# In plan() for CIRC paths:
constraints = _build_circ_constraints(path_dto)
self._planning_component.set_path_constraints(constraints)
# ... plan ...
# After planning (always clear constraints):
self._planning_component.set_path_constraints(Constraints())
```

[VERIFIED from PILZ docs: `path_constraints/name` = `'interim'` or `'center'`]
[VERIFIED from moveit2/moveit_py planning_component.cpp: `.def("set_path_constraints", ...)`]

### Pattern 4: Plan + Execute Result Handling (VERIFIED)

```python
# Source: github.com/moveit/moveit2/moveit_py/moveit/planning.pyi
# + moveit_py/src/moveit/moveit_core/controller_manager/controller_manager.cpp bindings

# Planning result
plan_result = planning_component.plan(single_plan_parameters=params)

if not plan_result:  # MotionPlanResponse __bool__ → True only on SUCCESS
    return PlanResultDTO(success=False, error_message=f'PILZ {planner_id} failed for {path_id}')

trajectory = plan_result.trajectory  # RobotTrajectory object

# Execution result
exec_status = moveit_instance.execute(trajectory, controllers=[])

if not exec_status:  # ExecutionStatus __bool__ → True only on SUCCEEDED
    return ...  # handle execution failure
```

[VERIFIED: moveit2/moveit_py/src/moveit/moveit_core/controller_manager/controller_manager.cpp L38-L56:
`def("__bool__", [](ExecStatus& s) { return static_cast<bool>(s); })`]
[VERIFIED: MoveItCpp::execute() → waitForExecution() → ExecutionStatus]

### Pattern 5: MoveItPy Connection with Timeout (RECOMMENDED)

```python
# on_configure in URMovementController:
import threading

timeout: float = self.get_parameter('moveit_connection_timeout').value

result_container: list = [None, None]  # [moveit_instance, exception]

def _init_moveit() -> None:
    try:
        result_container[0] = MoveItPy(node_name='moveit_py_node')
    except Exception as e:  # noqa: BLE001
        result_container[1] = e

init_thread = threading.Thread(target=_init_moveit, daemon=True)
init_thread.start()
init_thread.join(timeout=timeout)

if init_thread.is_alive():
    self.get_logger().error(
        f'MoveItPy failed to connect within {timeout}s timeout'
    )
    return TransitionCallbackReturn.FAILURE

if result_container[1] is not None:
    self.get_logger().error(f'MoveItPy connection failed: {result_container[1]}')
    return TransitionCallbackReturn.FAILURE

self._moveit: MoveItPy = result_container[0]
```

**Rationale:** `MoveItPy.__init__` is blocking. Wrapping in a thread with `join(timeout=)` is the correct pattern to honour `moveit_connection_timeout` without blocking `on_configure` indefinitely.

### Pattern 6: Mocking MoveItPy in Tests (VERIFIED approach)

```python
# Source: .github/rules/testing.md + moveit2 binding structure
# tests/unit/test_pilz_planner_service.py

import pytest
from unittest.mock import MagicMock, patch, PropertyMock

@pytest.fixture()
def mock_planning_component():
    """Mock moveit_py PlanningComponent."""
    arm = MagicMock()

    # Mock successful plan result (truthy)
    plan_result = MagicMock()
    plan_result.__bool__ = MagicMock(return_value=True)
    plan_result.trajectory = MagicMock()
    arm.plan.return_value = plan_result

    return arm


@pytest.fixture()
def mock_moveit(mocker):
    """Patch MoveItPy in the ur_movement_controller module."""
    mock_cls = mocker.patch('movement_controller.ur_movement_controller.MoveItPy')
    mock_instance = MagicMock()
    mock_cls.return_value = mock_instance

    # Planning component mock
    mock_arm = MagicMock()
    mock_instance.get_planning_component.return_value = mock_arm

    # Plan result mock (truthy = success)
    plan_result = MagicMock()
    plan_result.__bool__ = MagicMock(return_value=True)
    plan_result.trajectory = MagicMock()
    mock_arm.plan.return_value = plan_result

    # Execute result mock (truthy = SUCCEEDED)
    exec_status = MagicMock()
    exec_status.__bool__ = MagicMock(return_value=True)
    mock_instance.execute.return_value = exec_status

    return mock_instance, mock_arm
```

**Key:** `PlanRequestParameters` also needs to be patched or it will throw during construction (it requires `moveit_cpp` node). Patch it as: `mocker.patch('movement_controller.services.pilz_planner_service.PlanRequestParameters')`.

### Pattern 7: PlanResultDTO (Internal Model)

```python
# movement_controller/models/plan_result_dto.py
# Pydantic v2, arbitrary_types_allowed=True (RobotTrajectory is not a Pydantic type)
from __future__ import annotations
from typing import Any, Optional
from pydantic import BaseModel, ConfigDict, Field

class PlanResultDTO(BaseModel):
    """Internal result of a PILZ planning call."""
    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    success: bool = Field(description='True if planning succeeded')
    trajectory: Optional[Any] = Field(
        default=None,
        description='RobotTrajectory from moveit_py; None on failure'
    )
    error_message: str = Field(
        default='',
        description='Human-readable error; empty on success'
    )
```

**Note:** Using `Any` for `trajectory` avoids importing `moveit.core.robot_trajectory.RobotTrajectory` (not installed in devcontainer) at module load time. Type annotation is sufficient for IDE; the actual type is checked at runtime.

### Anti-Patterns to Avoid

- **`from moveit.planning import PlanningComponent as PlanningComponentType`** — use `Any` or string annotations for type hints in signatures; `PlanningComponent` import may fail at module load if `moveit_py` not installed.
- **Forgetting to clear path constraints after CIRC planning** — PILZ path constraints persist on `PlanningComponent.current_path_constraints_`. Always call `set_path_constraints(Constraints())` after CIRC to prevent contaminating the next LIN/PTP plan.
- **Using `MoveItPy(node_name=...)` without post-`rclpy.init()`** — MoveItPy must be instantiated **after** `rclpy.init()` is called. This is satisfied by `on_configure` (the executor is already spinning).
- **Blocking `send_goal` pattern** — Phase 3 uses `moveit.execute(trajectory, controllers=[])` which is a blocking call on the `MoveItPy` side (uses `waitForExecution()` internally). This is intentional for Phase 3 single-path execution. It is safe because `execute_callback` runs in a ReentrantCallbackGroup (separate thread).
- **Passing `blocking=True` as keyword arg to `execute()`** — the C++ binding signature is `execute(robot_trajectory, controllers)`. There is NO `blocking` parameter in the Python binding; execution is always blocking (calls `waitForExecution()`). The `blocking=True` in CONTEXT.md D-09 refers to the semantic intent, not a Python kwarg.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Trapezoidal LIN velocity profile | Custom Cartesian interpolation | PILZ LIN planner | Handles joint limits, singularities, orientation slerp |
| PTP joint-space motion | Custom joint interpolation | PILZ PTP planner | Fully synchronized trapezoidal joint profiles |
| Circular arc trajectory | Custom arc parameterization | PILZ CIRC planner | Handles interim/center modes, Cartesian limits, joint limit checks |
| Execute trajectory on controllers | Custom controller action client | `MoveItPy.execute()` | Handles controller selection, dispatch, and `waitForExecution()` |

**Key insight:** PILZ determinism comes from its internal algorithms. Any hand-rolled approximation will differ at joint limits and near singularities.

---

## Common Pitfalls

### Pitfall 1: Path Constraints Not Cleared After CIRC
**What goes wrong:** After a CIRC plan, `PlanningComponent.current_path_constraints_` still holds the `{name: 'interim', position_constraints: [...]}` constraint. The next LIN or PTP `plan()` call sends this CIRC constraint to the planner → planning fails with an unexpected constraint error.
**Why it happens:** `set_goal_state` replaces goal constraints, but `set_path_constraints` sets a separate field. Calling `plan()` again without clearing path constraints reuses them.
**How to avoid:** Always call `set_path_constraints(Constraints())` after every CIRC `plan()` call, even on failure. Best placed in a `try/finally` block inside `PilzPlannerService.plan()`.
**Warning signs:** LIN/PTP plans fail after a CIRC plan with cryptic planning errors.

### Pitfall 2: `PlanRequestParameters` Requires `MoveItPy` Instance
**What goes wrong:** `PlanRequestParameters(moveit, '')` requires an active `MoveItPy` instance. In tests, if `MoveItPy` is not mocked before `PilzPlannerService` is instantiated, the test will try to import `moveit.planning` and fail.
**Why it happens:** Python binding constructor calls `moveit_cpp->getNode()` to declare parameters.
**How to avoid:** In tests, mock `PlanRequestParameters` as well as `MoveItPy`. The simplest approach: mock both at the module level where imported.
**Warning signs:** `ModuleNotFoundError: No module named 'moveit'` in test collection.

### Pitfall 3: MoveItPy Not Installed in Devcontainer
**What goes wrong:** `from moveit.planning import MoveItPy` at module load time → `ModuleNotFoundError`. Even though `package.xml` declares `<exec_depend>moveit_py</exec_depend>`, the package is not installed in the devcontainer image.
**Why it happens:** The Dockerfile does not install `ros-jazzy-moveit`.
**How to avoid:** Add `ros-jazzy-moveit` to the Dockerfile and rebuild the devcontainer. This MUST be done before any Phase 3 code can be tested.
**Warning signs:** Import errors on `colcon build` or `colcon test`.

### Pitfall 4: `execute()` Python Signature Has No `blocking` Parameter
**What goes wrong:** Calling `moveit.execute(trajectory, blocking=True, controllers=[])` → TypeError (unexpected keyword argument `blocking`).
**Why it happens:** The Python binding for `MoveItCpp.execute` takes `(robot_trajectory, controllers)` only — no `blocking` param. The C++ deprecated signature with `blocking` is not exposed in Python.
**How to avoid:** Use `moveit.execute(trajectory, controllers=[])`. All MoveItPy execution is blocking (internally calls `waitForExecution()`).
**Warning signs:** `TypeError: execute() got an unexpected keyword argument 'blocking'`.

### Pitfall 5: CIRC `circ_type` Validation Must Happen in `goal_callback`
**What goes wrong:** If CIRC validation is deferred to `execute_callback`, the goal is accepted and then immediately aborted — callers see an unexpected abort instead of a clean rejection.
**Why it happens:** Phase 2 established the pattern: structural validation → reject before accept.
**How to avoid:** Validate CIRC `circ_type` in `_goal_callback` alongside the existing motion_type / path_id checks (D-11, D-12).
**Warning signs:** Clients see `GoalResponse.ACCEPT` followed immediately by `aborted` result.

### Pitfall 6: Speed/Acceleration Field Interpretation
**What goes wrong:** `TrajectoryPathDTO.cartesian_speed` is in m/s (absolute), but PILZ `max_velocity_scaling_factor` is relative (0.0–1.0). Passing a raw m/s value > 1.0 as a scaling factor results in clamping or errors.
**Why it happens:** The DTO spec says "m/s"; PILZ expects fractional scaling.
**How to avoid (Phase 3 approach):** If `cartesian_speed == 0.0` (default), use a safe default scaling factor (e.g., `0.1`). If `0.0 < cartesian_speed <= 1.0`, pass through as scaling factor. If `> 1.0`, clamp to `1.0` and log a WARNING. Proper absolute-to-relative conversion is a Phase 5 (CON-05) concern.
**Warning signs:** `cartesian_speed=2.0` passed as scaling factor silently gets treated as `1.0` (full speed).

---

## Code Examples

### Full `PilzPlannerService.plan()` Call Sequence
```python
# Source: verified patterns from official MoveIt2 Python API tutorial +
#         github.com/moveit/moveit2/moveit_py

from moveit.planning import MoveItPy, PlanRequestParameters  # noqa: E402
from moveit_msgs.msg import Constraints                        # always import

class PilzPlannerService:
    _PIPELINE: str = 'pilz_industrial_motion_planner'
    _PLANNER_MAP: dict[str, str] = {
        'LIN': 'LIN',
        'PTP': 'PTP',
        'CIRC': 'CIRC',
    }

    def __init__(self, planning_component, moveit_instance) -> None:
        self._planning_component = planning_component
        self._moveit = moveit_instance

    def plan(self, path_dto) -> PlanResultDTO:
        planner_id = self._PLANNER_MAP[path_dto.motion_type.value]

        self._planning_component.set_start_state_to_current_state()
        self._planning_component.set_goal_state(
            pose_stamped_msg=path_dto.target_pose,
            pose_link=path_dto.tool_frame or 'tool0',
        )

        if planner_id == 'CIRC':
            circ_constraints = self._build_circ_constraints(path_dto)
            self._planning_component.set_path_constraints(circ_constraints)

        try:
            params = PlanRequestParameters(self._moveit, '')
            params.planner_id = planner_id
            params.planning_pipeline = self._PIPELINE
            params.planning_attempts = 1
            params.planning_time = 5.0
            params.max_velocity_scaling_factor = self._to_scaling(path_dto.cartesian_speed)
            params.max_acceleration_scaling_factor = self._to_scaling(path_dto.acceleration)

            plan_result = self._planning_component.plan(single_plan_parameters=params)

            if not plan_result:
                return PlanResultDTO(success=False, error_message=f'{planner_id} planning failed')
            return PlanResultDTO(success=True, trajectory=plan_result.trajectory)

        finally:
            # Always clear path constraints — prevent contaminating next plan call
            if planner_id == 'CIRC':
                self._planning_component.set_path_constraints(Constraints())
```

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `MoveIt Commander` | `moveit_py` (`MoveItPy`) | ROS2 / MoveIt2 release | MoveIt Commander is ROS1 only; forbidden here |
| `planning_component.set_planner_id('LIN')` | `PlanRequestParameters.planner_id = 'LIN'` + `planning_pipeline = 'pilz_...'` | `moveit_cpp` API design | Pipeline + planner_id are both required in MoveIt2 |
| `planning_component.execute()` (PlanningComponent.execute) | `moveit_instance.execute(trajectory, controllers=[])` (MoveItCpp.execute) | Deprecated in moveit_cpp | `PlanningComponent.execute(blocking)` is deprecated; use `MoveItCpp.execute()` |

**Deprecated/outdated:**
- `PlanningComponent.execute(blocking)`: The C++ method exists but is `[[deprecated("Use MoveItCpp::execute()")]]`. Python binding exposes only `MoveItCpp.execute()`. [VERIFIED: moveit_cpp/include/moveit/moveit_cpp/planning_component.hpp L209-L215]

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `PlanRequestParameters(moveit_instance, '')` with empty namespace uses safe defaults (planner_id='', etc.) and allows direct attribute assignment | Pattern 1 | If empty string namespace causes parameter declaration errors, tests/configure may fail |
| A2 | The `planning.pyi` stub types for `PlanningComponent` are importable as `from moveit.planning import PlanningComponent` | Anti-patterns | If only importable from a C-extension internal, type annotations would need `TYPE_CHECKING` guard |
| A3 | `MoveItPy.__init__` raises an exception on connection failure/timeout (not just hangs forever) | Pattern 5 | If it hangs silently, the threading approach is needed to avoid infinite block |
| A4 | Speed scaling from `cartesian_speed` (m/s) → passed as 0-1 fraction in Phase 3 is safe since PILZ respects robot velocity limits and will not exceed them | Common Pitfalls §6 | If robot doesn't have safety limits enforced, a scaling factor of 1.0 may be too fast |

---

## Open Questions

1. **`PlanRequestParameters` constructor with `MoveItPy` instance in `PilzPlannerService`**
   - What we know: Constructor takes `(moveit_cpp_instance, namespace)`. `PilzPlannerService` receives only the `planning_component`, not `MoveItPy` itself (D-06 says "receives the planning component via its constructor").
   - What's unclear: If the service doesn't have the `MoveItPy` instance, it can't construct `PlanRequestParameters`. Either D-06 needs amending to inject `MoveItPy` too, OR `PilzPlannerService` constructs `PlanRequestParameters` using `PlanRequestParameters.__new__` / direct struct creation without the node.
   - **Recommendation:** Amend the service interface to accept both `planning_component` AND `moveit_instance`. D-09 already shows the controller holds `self._moveit` — pass it to the service at construction. The planner decides at planning time.

2. **PILZ planner needs pre-declared pipeline in MoveItPy YAML config**
   - What we know: MoveItPy loads `planning_pipelines.pipeline_names` from its ROS parameter YAML at startup. If `pilz_industrial_motion_planner` is not in `pipeline_names`, the pipeline cannot be used.
   - What's unclear: Does `ur_moveit_config` ship a PILZ-enabled `moveit_py` config, or does Phase 3 need to provide one?
   - **Recommendation:** Plan 1 should include creating/verifying a `moveit_py_config.yaml` that declares `pipeline_names: ["pilz_industrial_motion_planner"]` — or investigate what `ur_moveit_config` ships by default.

3. **`ur_moveit_config` ships with `pilz_cartesian_limits.yaml`?**
   - What we know: PILZ LIN/CIRC requires `robot_description_planning.cartesian_limits.*` parameters to be set.
   - What's unclear: Whether `ur_moveit_config` includes these or they need to be declared manually.
   - **Recommendation:** Check `ur_moveit_config` package contents after installation; if missing, Plan 2 must add a `pilz_cartesian_limits.yaml` config file.

---

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| `ros-jazzy-moveit` | Plans 1–4 (all) | ✗ | — | Must install; no fallback |
| `ros-jazzy-moveit-py` | All Python plans | ✗ | — | Included in `ros-jazzy-moveit` |
| `ros-jazzy-pilz-industrial-motion-planner` | Plan 2 (PILZ planner) | ✗ | — | Bundled in `ros-jazzy-moveit` |
| ROS2 apt source | Dockerfile apt-get | ✓ | noble/main | Already configured in ros2.sources |
| `pytest-mock` (`mocker` fixture) | Plan 4 (tests) | ✓ | in requirements-dev.txt | — |

**Missing dependencies with no fallback:**
- `ros-jazzy-moveit`: Must be added to Dockerfile before any Phase 3 code can be built or tested. Add: `RUN apt-get update && apt-get install -y ros-jazzy-moveit && rm -rf /var/lib/apt/lists/*`

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest + ament_pytest |
| Config file | `setup.cfg` (already configured with `--import-mode=importlib`) |
| Quick run command | `python -m pytest src/movement_controller/tests/ -v -k "pilz"` |
| Full suite command | `colcon test --packages-select movement_controller && colcon test-result --verbose` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| MOT-01 | PILZ pipeline maps LIN/PTP/CIRC to correct `pipeline_id` + `planner_id` | unit | `pytest tests/unit/test_pilz_planner_service.py -v` | ❌ Wave 0 |
| MOT-01 | PILZ plan() returns success/failure PlanResultDTO | unit | `pytest tests/unit/test_pilz_planner_service.py -v` | ❌ Wave 0 |
| MOT-01 | CIRC path builds correct `Constraints` message (name='interim'/'center') | unit | `pytest tests/unit/test_pilz_planner_service.py -v` | ❌ Wave 0 |
| MOT-05 | Goal rejected while executing (already covered by Phase 2) | unit | `pytest tests/unit/test_ur_movement_controller.py -v` | ✅ exists |
| MOT-02 | Single-path execute sends executing+completed feedback pair | integration | `pytest tests/integration/test_execute_trajectory_smoke.py -v` | ❌ Wave 0 |
| D-11 | CIRC with empty circ_type rejected in goal_callback | unit | `pytest tests/unit/test_ur_movement_controller.py -k circ` | ❌ Wave 0 |
| D-16 | Planning failure aborts goal with error message | unit/integration | `pytest tests/unit/test_pilz_planner_service.py -k plan_fail` | ❌ Wave 0 |

### Sampling Rate
- **Per task commit:** `python -m pytest tests/unit/ -v`
- **Per wave merge:** `colcon test --packages-select movement_controller`
- **Phase gate:** Full suite green before `/gsd-verify-work`

### Wave 0 Gaps
- [ ] `tests/unit/test_pilz_planner_service.py` — covers MOT-01, D-07, D-16
- [ ] `tests/integration/test_execute_trajectory_smoke.py` — covers MOT-02 (D-19 approach)
- [ ] CIRC test cases in `tests/unit/test_ur_movement_controller.py` — covers D-11/D-12

*(Existing `tests/unit/test_ur_movement_controller.py` exists and covers Phase 2 goals; extend in Phase 3.)*

---

## Security Domain

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | — |
| V3 Session Management | no | — |
| V4 Access Control | no | — |
| V5 Input Validation | yes | Pydantic v2 models; `CircTypeEnum` validation in `goal_callback` |
| V6 Cryptography | no | — |

### Known Threat Patterns for ROS2 Action Servers

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Malformed CIRC constraints (empty `circ_type`) | Tampering | `goal_callback` rejects before accept |
| Goal flood while executing | Denial of Service | `_is_executing` lock + `GoalResponse.REJECT` (Phase 2) |
| Oversized trajectory paths (malicious goal) | Tampering | Pydantic DTO validation; PILZ planner rejects infeasible goals |

---

## Sources

### Primary (HIGH confidence)
- [CITED: moveit.picknik.ai/main/.../motion_planning_python_api] — `PlanningComponent` API: `set_start_state_to_current_state()`, `set_goal_state(pose_stamped_msg, pose_link)`, `plan(single_plan_parameters)`, `set_path_constraints()`
- [CITED: moveit.picknik.ai/main/.../pilz_industrial_motion_planner] — PILZ interface: `planner_id = 'LIN'|'PTP'|'CIRC'`, CIRC `path_constraints.name`, `path_constraints.position_constraints.constraint_region.primitive_poses`
- [VERIFIED: github.com/moveit/moveit2/moveit_py/moveit/planning.pyi] — Python stubs: `PlanRequestParameters.planning_pipeline`, `.planner_id`, `.planning_time`, etc. `MoveItPy.execute(robot_trajectory, controllers)`; `ExecutionStatus.__bool__`
- [VERIFIED: github.com/moveit/moveit2/moveit_py/src/moveit/moveit_ros/moveit_cpp/planning_component.cpp] — `PlanRequestParameters` binding, `planning_pipeline` / `planner_id` attributes
- [VERIFIED: github.com/moveit/moveit2/moveit_ros/planning/moveit_cpp/src/planning_component.cpp] — `getMotionPlanRequest`: `request.pipeline_id = plan_request_parameters.planning_pipeline`, `request.planner_id = plan_request_parameters.planner_id`
- [VERIFIED: github.com/moveit/moveit2/moveit_ros/planning/moveit_cpp/src/moveit_cpp.cpp L206-L227] — `MoveItCpp::execute()` signature; calls `waitForExecution()`; deprecation of blocking param
- [VERIFIED: github.com/moveit/moveit2/moveit_py/src/moveit/moveit_core/controller_manager/controller_manager.cpp] — `ExecutionStatus.__bool__` Python binding
- [VERIFIED: rosdep resolve moveit_py output in devcontainer] — `ros-jazzy-moveit-py` package exists in ROS2 Jazzy apt repository

### Secondary (MEDIUM confidence)
- [CITED: github.com/moveit/moveit2/moveit_py/moveit/planning.pyi L12-L69] — `PlanRequestParameters.__init__`, `MoveItPy` class interface

---

## Metadata

**Confidence breakdown:**
- PILZ planner pipeline/planner_id names: HIGH — verified from both official docs and GitHub source
- `PlanRequestParameters` attribute names: HIGH — verified from `.pyi` stub and C++ bindings
- `MoveItPy.execute()` return type and semantics: HIGH — verified from C++ source + Python binding
- CIRC `path_constraints` structure: HIGH — verified from PILZ official docs
- MoveItPy connection timeout approach (threading): MEDIUM — pattern derived from known blocking behaviour; exception-on-timeout not explicitly confirmed
- `PlanRequestParameters` constructor behaviour with empty namespace: MEDIUM — derived from C++ `load()` source code defaults

**Research date:** 2026-05-28
**Valid until:** 2026-06-28 (stable; MoveIt2 APIs don't change rapidly)
