# Phase 3: MoveIt2 + PILZ Single-Path Execution — Pattern Map

**Mapped:** 2026-05-28
**Files analyzed:** 7 (2 new service/model files + 1 new integration test + 1 new unit test + 3 file modifications)
**Analogs found:** 7 / 7

---

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|-------------------|------|-----------|----------------|---------------|
| `movement_controller/ur_movement_controller.py` | controller | event-driven | itself (Phase 2) | exact — extend only |
| `movement_controller/services/pilz_planner_service.py` | service | request-response | `movement_controller/utils/trajectory_grouper.py` | role-match (plain Python class, clear return type) |
| `movement_controller/models/plan_result_dto.py` | model | transform | `movement_controller/models/trajectory_path_dto.py` | exact (Pydantic DTO, `arbitrary_types_allowed`, `frozen`) |
| `movement_controller/models/trajectory_path_dto.py` | model | transform | itself (Phase 2) | exact — add CIRC validator only |
| `docker/Dockerfile` | config | N/A | itself (Phase 2) | exact — add one apt package block |
| `tests/unit/test_pilz_planner_service.py` | test | request-response | `tests/unit/test_enums_and_dtos.py` | exact (pytest, no rclpy, fixture-based) |
| `tests/integration/test_moveit_execution.py` | test | event-driven | `tests/unit/test_ur_movement_controller.py` | role-match (asyncio + MagicMock goal handle, same node fixture) |

---

## Pattern Assignments

### `movement_controller/ur_movement_controller.py` (controller, event-driven — MODIFY)

**Analog:** itself — [src/movement_controller/movement_controller/ur_movement_controller.py](src/movement_controller/movement_controller/ur_movement_controller.py)

**Imports pattern** (lines 1–18 of the file, after license header):
```python
"""URMovementController — ROS2 LifecycleNode for UR robot trajectory execution."""

from threading import Lock

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from movement_controller.action import ExecuteTrajectory
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
```

**New imports to add** (Phase 3 additions — place after existing imports):
```python
import threading

from moveit.planning import MoveItPy, PlanRequestParameters

from movement_controller.services.pilz_planner_service import PilzPlannerService
```

**`__init__` pattern** (lines 50–55 — add new fields after `_executing_lock`):
```python
    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._is_active: bool = False
        self._is_executing: bool = False
        self._executing_lock: Lock = Lock()
        # Phase 3 additions:
        self._moveit: MoveItPy | None = None
        self._planner_service: PilzPlannerService | None = None
```

**`on_configure` parameter declaration pattern** (lines 60–75 — copy the existing `declare_parameter` block and append):
```python
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')

        self.declare_parameter(
            'action_server_name',
            'movement_controller/execute_trajectory',
            ParameterDescriptor(description='ROS2 action server name for ExecuteTrajectory action'),
        )
        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name (used from Phase 3 onward)'),
        )
        # Phase 3: add this block immediately after the existing declare_parameter calls:
        self.declare_parameter(
            'moveit_connection_timeout',
            10.0,
            ParameterDescriptor(description='Seconds to wait for MoveItPy to connect before failing on_configure'),
        )
```

**MoveItPy init-with-timeout pattern** (Research Pattern 5 — place after parameter declarations, before action server creation):
```python
        timeout: float = self.get_parameter('moveit_connection_timeout').value
        result_container: list = [None, None]   # [moveit_instance, exception]

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

        self._moveit = result_container[0]
        moveit_group_name = self.get_parameter('moveit_group_name').value
        planning_component = self._moveit.get_planning_component(moveit_group_name)
        self._planner_service = PilzPlannerService(self._moveit, planning_component)
        self.get_logger().info(f'MoveItPy connected; planning group: {moveit_group_name}')
```

**`on_cleanup` pattern** (lines 100–107 — mirror how `_action_server` is destroyed, add `_moveit` teardown):
```python
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        # Phase 3 additions:
        self._moveit = None
        self._planner_service = None
        return TransitionCallbackReturn.SUCCESS
```

**`_goal_callback` CIRC validation pattern** (lines 112–140 — insert between DTO validation and ACCEPT):
```python
        # 3. Full goal validation via DTO — reset executing flag on failure
        try:
            goal_dto = TrajectoryGoalDTO.from_ros_msg(goal)
        except (ValidationError, ValueError) as e:
            self.get_logger().error(f'Goal rejected: {e}')
            with self._executing_lock:
                self._is_executing = False
            return GoalResponse.REJECT

        # 4. Phase 3: CIRC path structural validation (D-11)
        for path in goal_dto.paths:
            if path.motion_type == MotionTypeEnum.CIRC:
                if path.circ_type not in (CircTypeEnum.INTERIM, CircTypeEnum.CENTER):
                    self.get_logger().error(
                        f'Goal rejected: path {path.path_id!r} has motion_type CIRC '
                        f'but invalid circ_type {path.circ_type!r}'
                    )
                    with self._executing_lock:
                        self._is_executing = False
                    return GoalResponse.REJECT

        return GoalResponse.ACCEPT
```
> Note: `TrajectoryPathDTO.from_ros_msg` already parses `circ_type` into `CircTypeEnum` and rejects unknown values via `ValueError`. The additional check in `_goal_callback` guards against a CIRC path with default `circ_type=INTERIM` where the ROS msg field was left empty — that is structurally valid but may not be intentional for CIRC. Per D-11, validation in `_goal_callback` is the explicit rejection point.

**`_execute_callback` flatten + plan + execute pattern** (lines 145–200 — replace the stub group loop):
```python
    async def _execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteTrajectory.Result:
        try:
            goal_dto = TrajectoryGoalDTO.from_ros_msg(goal_handle.request)
            groups = TrajectoryGrouper.group(goal_dto.paths)
            completed_ids: list[str] = []

            # Phase 3: flatten groups to individual paths (D-13); Phase 4 replaces with MoveGroupSequence
            for group in groups:
                for path in group:
                    # Publish executing feedback (D-15)
                    fb = ExecuteTrajectory.Feedback()
                    fb.status = FeedbackStatusEnum.EXECUTING.value
                    fb.trajectory_path_ids = [path.path_id]
                    goal_handle.publish_feedback(fb)

                    # Plan via PilzPlannerService (D-16: fail-fast on planning failure)
                    plan_result = self._planner_service.plan(path)
                    if not plan_result.success:
                        self.get_logger().error(
                            f'Planning failed for path {path.path_id!r}: {plan_result.error_message}'
                        )
                        result = ExecuteTrajectory.Result()
                        result.success = False
                        result.error_message = plan_result.error_message
                        goal_handle.abort()
                        return result

                    # Execute trajectory (D-17: fail-fast on execution failure)
                    exec_status = self._moveit.execute(
                        plan_result.trajectory, blocking=True, controllers=[]
                    )
                    if not exec_status:
                        err = f'Execution failed for path {path.path_id!r}'
                        self.get_logger().error(err)
                        result = ExecuteTrajectory.Result()
                        result.success = False
                        result.error_message = err
                        goal_handle.abort()
                        return result

                    # Publish completed feedback (D-15)
                    fb2 = ExecuteTrajectory.Feedback()
                    fb2.status = FeedbackStatusEnum.COMPLETED.value
                    fb2.trajectory_path_ids = [path.path_id]
                    goal_handle.publish_feedback(fb2)
                    completed_ids.append(path.path_id)

            result = ExecuteTrajectory.Result()
            result.success = True
            result.error_message = ''
            result.trajectory_paths_completed = completed_ids
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

        finally:
            with self._executing_lock:
                self._is_executing = False
```

---

### `movement_controller/services/pilz_planner_service.py` (service, request-response — CREATE)

**Analog:** [src/movement_controller/movement_controller/utils/trajectory_grouper.py](src/movement_controller/movement_controller/utils/trajectory_grouper.py) (plain Python class, clear return type, no ROS2 node)

**License header pattern** (lines 1–28 of trajectory_grouper.py — identical BSD-3-Clause block, copy verbatim, change module docstring):
```python
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# [... full BSD-3-Clause text ...]
"""PilzPlannerService — maps TrajectoryPathDTO to PILZ planner parameters and executes planning."""
```

**Imports pattern** (based on Research patterns 1–3):
```python
from __future__ import annotations

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
```

**Class + constructor pattern** (mirrors TrajectoryGrouper's simplicity, but with injected dependencies):
```python
_PILZ_PIPELINE = 'pilz_industrial_motion_planner'
_PLANNER_IDS: dict[MotionTypeEnum, str] = {
    MotionTypeEnum.LIN: 'LIN',
    MotionTypeEnum.PTP: 'PTP',
    MotionTypeEnum.CIRC: 'CIRC',
}

class PilzPlannerService:
    """Plain Python service that plans a single trajectory path via the PILZ planner."""

    def __init__(self, moveit: MoveItPy, planning_component) -> None:
        # planning_component: moveit.planning.PlanningComponent (type-hint omitted to avoid import at class level)
        self._moveit = moveit
        self._planning_component = planning_component
```

**`plan()` core pattern** (Research Patterns 1–4):
```python
    def plan(self, path_dto: TrajectoryPathDTO) -> PlanResultDTO:
        """Plan a single path using the PILZ planner. Returns PlanResultDTO."""
        planner_id = _PLANNER_IDS[path_dto.motion_type]

        self._planning_component.set_start_state_to_current_state()
        self._planning_component.set_goal_state(
            pose_stamped_msg=path_dto.target_pose,
            pose_link=path_dto.tool_frame or 'tool0',
        )

        if path_dto.motion_type == MotionTypeEnum.CIRC:
            constraints = self._build_circ_constraints(path_dto)
            self._planning_component.set_path_constraints(constraints)

        try:
            params = PlanRequestParameters(self._moveit, '')
            params.planner_id = planner_id
            params.planning_pipeline = _PILZ_PIPELINE
            params.planning_attempts = 1
            params.planning_time = 5.0
            params.max_velocity_scaling_factor = 0.1    # Phase 3: m/s→scaling deferred to Phase 5 (CON-05)
            params.max_acceleration_scaling_factor = 0.1  # Phase 3: m/s²→scaling deferred to Phase 5

            plan_result = self._planning_component.plan(single_plan_parameters=params)
        finally:
            # Always clear path constraints (even on exception) to avoid leaking state
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                self._planning_component.set_path_constraints(Constraints())

        if not plan_result:
            return PlanResultDTO(
                success=False,
                error_message=f'PILZ {planner_id} planning failed for path {path_dto.path_id!r}',
            )

        return PlanResultDTO(success=True, trajectory=plan_result.trajectory)
```

**CIRC constraint helper** (Research Pattern 3):
```python
    @staticmethod
    def _build_circ_constraints(path_dto: TrajectoryPathDTO) -> Constraints:
        constraints = Constraints()
        constraints.name = path_dto.circ_type.value   # 'interim' or 'center'

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = path_dto.target_pose.header.frame_id
        pos_constraint.link_name = path_dto.tool_frame or 'tool0'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]

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
```

---

### `movement_controller/models/plan_result_dto.py` (model, transform — CREATE)

**Analog:** [src/movement_controller/movement_controller/models/trajectory_path_dto.py](src/movement_controller/movement_controller/models/trajectory_path_dto.py)

**License header:** identical BSD-3-Clause block; change module docstring to `"""PlanResultDTO — internal result type for PilzPlannerService.plan()."""`

**Imports pattern**:
```python
from __future__ import annotations

from moveit.core.robot_trajectory import RobotTrajectory
from pydantic import BaseModel, ConfigDict, Field
```

**Core model pattern** (`arbitrary_types_allowed=True` because `RobotTrajectory` is not a native Pydantic type; `frozen=True` for immutability):
```python
class PlanResultDTO(BaseModel):
    """Internal result from PilzPlannerService.plan() — not a ROS2 message."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    success: bool = Field(description='True if PILZ planning succeeded')
    trajectory: RobotTrajectory | None = Field(
        default=None,
        description='RobotTrajectory object returned by MoveItPy; None on failure',
    )
    error_message: str = Field(
        default='',
        description='Human-readable error description; empty on success',
    )
```

> `RobotTrajectory` is imported directly from `moveit.core.robot_trajectory`. MoveIt2 (`ros-jazzy-moveit`) is always installed in the devcontainer via `rosdep` (`startup.sh`), so this import is guaranteed to succeed. No `try/except ImportError` guard is needed.

---

### `movement_controller/models/trajectory_path_dto.py` (model, transform — MODIFY)

**Analog:** itself — [src/movement_controller/movement_controller/models/trajectory_path_dto.py](src/movement_controller/movement_controller/models/trajectory_path_dto.py)

**What changes:** The CIRC validation in `from_ros_msg` already rejects unknown `circ_type` values (lines 105–111). Per D-12, no structural change to the DTO is required. Any CIRC `circ_type` validation in `_goal_callback` is inline in the controller. This file **may not need modification** — the planner decides based on whether inline controller validation is sufficient (D-12 says either is acceptable).

**Existing `from_ros_msg` pattern for reference** (lines 100–128):
```python
    @classmethod
    def from_ros_msg(cls, ros_msg: TrajectoryPath) -> TrajectoryPathDTO:
        try:
            motion_type = MotionTypeEnum(ros_msg.motion_type)
        except ValueError:
            raise ValueError(f'Invalid motion_type: {ros_msg.motion_type!r}')

        try:
            if ros_msg.circ_type == '' or ros_msg.circ_type is None:
                circ_type = CircTypeEnum.INTERIM
            else:
                circ_type = CircTypeEnum(ros_msg.circ_type)
        except ValueError:
            raise ValueError(f'Invalid circ_type: {ros_msg.circ_type!r}')

        return cls(
            path_id=ros_msg.path_id,
            motion_type=motion_type,
            target_pose=ros_msg.target_pose,
            blend_radius=ros_msg.blend_radius,
            cartesian_speed=ros_msg.cartesian_speed,
            acceleration=ros_msg.acceleration,
            tool_frame=ros_msg.tool_frame,
            circ_type=circ_type,
            circ_point=ros_msg.circ_point,
        )
```

---

### `docker/Dockerfile` (config — MODIFY)

**Analog:** itself — [docker/Dockerfile](docker/Dockerfile)

**Existing apt block pattern** (lines 6–13 — append `ros-jazzy-moveit` to this block):
```dockerfile
# ── System dependencies ────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-moveit \
    sudo \
    && rm -rf /var/lib/apt/lists/*
```

> `ros-jazzy-moveit` is the apt meta-package that includes `moveit_py`, `pilz_industrial_motion_planner`, and all MoveIt2 Python bindings. It must be added **before** any ROS2 Python venv setup so the venv's `--system-site-packages` can see the installed bindings.

---

### `tests/unit/test_pilz_planner_service.py` (test, request-response — CREATE)

**Analog:** [src/movement_controller/tests/unit/test_enums_and_dtos.py](src/movement_controller/tests/unit/test_enums_and_dtos.py) (no rclpy, fixture-based, pytest-only)

**License header:** identical BSD-3-Clause; change docstring to `"""Unit tests for PilzPlannerService — all MoveItPy dependencies mocked."""`

**Imports pattern** (lines 31–38 of test_enums_and_dtos.py):
```python
import pytest
from unittest.mock import MagicMock
from geometry_msgs.msg import Point, PoseStamped

from movement_controller.enums.circ_type_enum import CircTypeEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService
```

**Mock fixture pattern** (Research Pattern 6, adapted to project style):
```python
_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'


def _make_path_dto(**overrides) -> TrajectoryPathDTO:
    defaults = {
        'path_id': _UUID,
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


@pytest.fixture()
def mock_planning_component():
    arm = MagicMock()
    plan_result = MagicMock()
    plan_result.__bool__ = MagicMock(return_value=True)
    plan_result.trajectory = MagicMock()
    arm.plan.return_value = plan_result
    return arm


@pytest.fixture()
def mock_moveit():
    return MagicMock()


@pytest.fixture()
def service(mock_moveit, mock_planning_component):
    return PilzPlannerService(mock_moveit, mock_planning_component)
```

**Core test pattern** (mirroring test_enums_and_dtos.py function naming):
```python
def test_plan_lin_success(service, mock_planning_component):
    """plan() returns PlanResultDTO(success=True) for a valid LIN path."""
    result = service.plan(_make_path_dto(motion_type=MotionTypeEnum.LIN))
    assert result.success is True
    assert result.trajectory is not None
    mock_planning_component.set_start_state_to_current_state.assert_called_once()


def test_plan_returns_failure_when_planning_component_fails(service, mock_planning_component):
    """plan() returns PlanResultDTO(success=False) when PlanningComponent.plan() returns falsy."""
    fail_result = MagicMock()
    fail_result.__bool__ = MagicMock(return_value=False)
    mock_planning_component.plan.return_value = fail_result

    result = service.plan(_make_path_dto())
    assert result.success is False
    assert 'LIN' in result.error_message
```

---

### `tests/integration/test_moveit_execution.py` (test, event-driven — CREATE)

**Analog:** [src/movement_controller/tests/unit/test_ur_movement_controller.py](src/movement_controller/tests/unit/test_ur_movement_controller.py) (asyncio + MagicMock goal handle, rclpy module fixture, node fixture)

**License header:** identical BSD-3-Clause; change docstring to `"""Integration smoke test — URMovementController with MoveItPy mocked at module level."""`

**Imports pattern** (lines 30–35 of test_ur_movement_controller.py):
```python
import asyncio
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import GoalResponse

from movement_controller.ur_movement_controller import URMovementController
```

**rclpy module fixture** (lines 44–48 of test_ur_movement_controller.py — copy exactly):
```python
@pytest.fixture(scope='module')
def ros_context():
    """Initialise rclpy once per test module."""
    rclpy.init()
    yield
    rclpy.shutdown()
```

**Node fixture with MoveItPy mock** (extends node fixture pattern from test_ur_movement_controller.py lines 51–56; patches `moveit.planning.MoveItPy` before node creation):
```python
@pytest.fixture
def node_with_moveit(ros_context):
    """URMovementController with MoveItPy mocked at module level (D-19)."""
    mock_moveit = MagicMock()
    mock_plan = MagicMock()
    mock_plan.__bool__ = MagicMock(return_value=True)
    mock_plan.trajectory = MagicMock()
    mock_arm = MagicMock()
    mock_arm.plan.return_value = mock_plan
    mock_moveit.get_planning_component.return_value = mock_arm
    mock_moveit.execute.return_value = MagicMock(__bool__=MagicMock(return_value=True))

    with patch('movement_controller.ur_movement_controller.MoveItPy', return_value=mock_moveit):
        n = URMovementController()
        n._is_active = True
        n._moveit = mock_moveit
        n._planner_service = MagicMock()
        n._planner_service.plan.return_value = MagicMock(
            success=True, trajectory=mock_plan.trajectory
        )
        yield n
    n.destroy_node()
```

**Goal helper** (lines 57–70 of test_ur_movement_controller.py — `_make_ros_goal` / `_make_path_msg` patterns):
```python
_UUID1 = '00000000-0000-4000-8000-000000000001'

def _make_path_msg(path_id: str, motion_type: str = 'LIN') -> MagicMock:
    m = MagicMock()
    m.path_id = path_id
    m.motion_type = motion_type
    m.blend_radius = 0.0
    m.target_pose = PoseStamped()
    m.circ_point = Point()
    m.cartesian_speed = 0.0
    m.acceleration = 0.0
    m.tool_frame = ''
    m.circ_type = 'interim'
    return m
```

**Core integration test** (mirrors test_execute_callback_stub_feedback_sequence structure, lines 143–158):
```python
def test_execute_trajectory_single_path_success(node_with_moveit):
    """1-path LIN goal → executing + completed feedback, result.success=True (D-19)."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1)]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is True
    assert result.trajectory_paths_completed == [_UUID1]
    assert mock_goal_handle.publish_feedback.call_count == 2  # executing + completed
    mock_goal_handle.succeed.assert_called_once()


def test_execute_trajectory_aborts_on_plan_failure(node_with_moveit):
    """Planning failure → goal abort, success=False, error_message contains path_id (D-16)."""
    node_with_moveit._planner_service.plan.return_value = MagicMock(
        success=False, error_message=f'PILZ LIN planning failed for path {_UUID1!r}'
    )
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1)]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.abort = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is False
    assert _UUID1 in result.error_message
    mock_goal_handle.abort.assert_called_once()
```

---

## Shared Patterns

### License Header
**Source:** Every existing source file (e.g., [src/movement_controller/movement_controller/ur_movement_controller.py](src/movement_controller/movement_controller/ur_movement_controller.py) lines 1–26)
**Apply to:** All new `.py` files created in Phase 3
```python
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
```

### Error Handling (controller boundary pattern)
**Source:** [src/movement_controller/movement_controller/ur_movement_controller.py](src/movement_controller/movement_controller/ur_movement_controller.py) lines 192–200 (the `except` block in `_execute_callback`)
**Apply to:** `_execute_callback` and all places where `PilzPlannerService.plan()` or `moveit.execute()` fails — log at ERROR before returning failure result
```python
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

        finally:
            with self._executing_lock:
                self._is_executing = False
```

### `declare_parameter` pattern
**Source:** [src/movement_controller/movement_controller/ur_movement_controller.py](src/movement_controller/movement_controller/ur_movement_controller.py) lines 62–73
**Apply to:** `moveit_connection_timeout` parameter declaration in `on_configure`
```python
        self.declare_parameter(
            'moveit_connection_timeout',
            10.0,
            ParameterDescriptor(description='Seconds to wait for MoveItPy to connect before failing on_configure'),
        )
```

### Pydantic DTO model config
**Source:** [src/movement_controller/movement_controller/models/trajectory_path_dto.py](src/movement_controller/movement_controller/models/trajectory_path_dto.py) lines 50–53
**Apply to:** `PlanResultDTO` (holds a `RobotTrajectory` which is not a pydantic-native type)
```python
    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)
```

### Enum class pattern
**Source:** [src/movement_controller/movement_controller/enums/motion_type_enum.py](src/movement_controller/movement_controller/enums/motion_type_enum.py) lines 31–36
**Apply to:** Any new enum values if needed in Phase 3
```python
class MotionTypeEnum(str, Enum):
    """Motion type constants for trajectory path planning."""
    LIN = "LIN"
    PTP = "PTP"
    CIRC = "CIRC"
```

---

## No Analog Found

All Phase 3 files have close analogs in the existing codebase. No files require falling back to RESEARCH.md patterns alone.

| File | Note |
|------|------|
| `services/pilz_planner_service.py` | The `moveit_py` API calls have no analog (new capability), but the *class structure* mirrors `trajectory_grouper.py`. All API call patterns are in RESEARCH.md §Patterns 1–4. |

---

## Metadata

**Analog search scope:** `src/movement_controller/movement_controller/`, `src/movement_controller/tests/`, `docker/`
**Files scanned:** 11 (ur_movement_controller.py, trajectory_path_dto.py, trajectory_goal_dto.py, trajectory_grouper.py, motion_type_enum.py, circ_type_enum.py, feedback_status_enum.py, services/__init__.py, test_ur_movement_controller.py, test_enums_and_dtos.py, Dockerfile)
**Pattern extraction date:** 2026-05-28
