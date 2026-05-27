# Phase 2: LifecycleNode & Action Server Skeleton — Pattern Map

**Mapped:** 2026-05-27
**Files analyzed:** 11 (9 new + 2 modified)
**Analogs found:** 4 / 11 (all production code is greenfield; test and config patterns from Phase 1)

---

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|-------------------|------|-----------|----------------|---------------|
| `movement_controller/ur_movement_controller.py` | controller (node) | event-driven (lifecycle + action) | `movement_controller/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `movement_controller/enums/motion_type_enum.py` | model (enum) | transform | `movement_controller/models/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `movement_controller/enums/feedback_status_enum.py` | model (enum) | transform | `movement_controller/models/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `movement_controller/models/trajectory_path_dto.py` | model (DTO) | transform | `movement_controller/models/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `movement_controller/models/trajectory_goal_dto.py` | model (DTO) | transform | `movement_controller/models/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `movement_controller/utils/trajectory_grouper.py` | utility | transform | `movement_controller/utils/__init__.py` (header only) | header-only — code from RESEARCH.md |
| `tests/unit/test_enums_and_dtos.py` | test | request-response | `tests/unit/test_imports.py` | role-match (exact structure) |
| `tests/unit/test_trajectory_grouper.py` | test | request-response | `tests/unit/test_imports.py` | role-match (exact structure) |
| `tests/unit/test_ur_movement_controller.py` | test | request-response | `tests/unit/test_imports.py` | role-match (exact structure) |
| `CMakeLists.txt` *(modified)* | config | — | `CMakeLists.txt` lines 77–91 | exact (add 3 more `ament_add_pytest_test` blocks) |
| `setup.py` *(modified)* | config | — | `setup.py` lines 51–56 | exact (populate `console_scripts`) |

---

## Pattern Assignments

### `movement_controller/ur_movement_controller.py` (controller, event-driven)

**Analog:** `movement_controller/__init__.py` (BSD-3 header); code pattern from RESEARCH.md Pattern 1–3 + 7 (all verified against rclpy Jazzy installed in devcontainer).

**BSD-3 header pattern** (`movement_controller/__init__.py` lines 1–28):
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

**Imports pattern** (RESEARCH.md Pattern 1 + 7):
```python
import threading

import rclpy
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, GoalResponse, ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from movement_controller.action import ExecuteTrajectory
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
```

**Node `__init__` pattern** (RESEARCH.md Pattern 1; copilot-instructions.md §ROS2 Node Patterns):
```python
class URMovementController(LifecycleNode):

    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._is_executing: bool = False
        self._executing_lock = threading.Lock()
```

**`on_configure` pattern** (RESEARCH.md Pattern 1 + 7; D-01):
```python
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        self.declare_parameter(
            'action_server_name',
            'movement_controller/execute_trajectory',
            ParameterDescriptor(description='ROS2 action server name')
        )
        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name')
        )
        action_server_name = self.get_parameter('action_server_name').value
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            action_server_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        return TransitionCallbackReturn.SUCCESS
```

**`on_activate` / `on_deactivate` / `on_cleanup` pattern** (RESEARCH.md Pattern 1):
```python
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        return TransitionCallbackReturn.SUCCESS
```

**`_goal_callback` pattern — CRITICAL: use `_state_machine.current_state[0]`, NOT `get_current_state()`** (RESEARCH.md Pattern 2 + Pitfall 1; D-12, D-13, D-14):
```python
    def _goal_callback(self, goal: ExecuteTrajectory.Goal) -> GoalResponse:
        # Check 1: lifecycle state (D-12, D-14)
        if self._state_machine.current_state[0] != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error(
                'Goal rejected: node not in ACTIVE state '
                f'(current: {self._state_machine.current_state[1]})'
            )
            return GoalResponse.REJECT
        # Check 2: concurrent execution guard (D-12, D-13)
        with self._executing_lock:
            if self._is_executing:
                self.get_logger().error('Goal rejected: another goal is already executing')
                return GoalResponse.REJECT
        # Check 3: structural validation (D-03)
        if not goal.paths:
            self.get_logger().error('Goal rejected: paths list is empty')
            return GoalResponse.REJECT
        for path in goal.paths:
            if not path.path_id:
                self.get_logger().error('Goal rejected: path_id is empty')
                return GoalResponse.REJECT
            if path.motion_type not in ('LIN', 'PTP', 'CIRC'):
                self.get_logger().error(f'Goal rejected: invalid motion_type {path.motion_type!r}')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT
```

**`_execute_callback` pattern** (RESEARCH.md Pattern 3; D-05, D-06, D-11, D-13; copilot-instructions.md §Error Handling):
```python
    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecuteTrajectory.Result:
        with self._executing_lock:
            self._is_executing = True
        try:
            paths = [TrajectoryPathDTO.from_ros_msg(p) for p in goal_handle.request.paths]
            groups = TrajectoryGrouper.group(paths)

            for group in groups:
                path_ids = [p.path_id for p in group]
                fb = ExecuteTrajectory.Feedback()
                fb.status = FeedbackStatusEnum.EXECUTING.value
                fb.trajectory_path_ids = path_ids
                goal_handle.publish_feedback(fb)
                fb2 = ExecuteTrajectory.Feedback()
                fb2.status = FeedbackStatusEnum.COMPLETED.value
                fb2.trajectory_path_ids = path_ids
                goal_handle.publish_feedback(fb2)

            result = ExecuteTrajectory.Result()
            result.success = True
            result.error_message = ''
            result.trajectory_paths_completed = [p.path_id for p in paths]
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

**`main()` entry point pattern** (rclpy convention):
```python
def main(args=None) -> None:
    rclpy.init(args=args)
    node = URMovementController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### `movement_controller/enums/motion_type_enum.py` (model/enum, transform)

**Analog:** `movement_controller/models/__init__.py` (BSD-3 header only); code from RESEARCH.md Pattern 5.

**Full file pattern** (RESEARCH.md Pattern 5; D-15; copilot-instructions.md §Data Models):
```python
# [BSD-3 header]
"""Motion type enumeration for trajectory paths."""

from enum import Enum


class MotionTypeEnum(str, Enum):
    """Valid motion types for a trajectory path.

    Values map directly to the MOTION_TYPE_* constants in TrajectoryPath.msg.
    """

    LIN = "LIN"
    PTP = "PTP"
    CIRC = "CIRC"
```

---

### `movement_controller/enums/feedback_status_enum.py` (model/enum, transform)

**Analog:** `movement_controller/models/__init__.py` (BSD-3 header only); code from RESEARCH.md Pattern 5.

**Full file pattern** (RESEARCH.md Pattern 5; D-15):
```python
# [BSD-3 header]
"""Feedback status enumeration for trajectory execution."""

from enum import Enum


class FeedbackStatusEnum(str, Enum):
    """Status values published as action feedback during execution."""

    EXECUTING = "executing"
    COMPLETED = "completed"
```

---

### `movement_controller/models/trajectory_path_dto.py` (model/DTO, transform)

**Analog:** `movement_controller/models/__init__.py` (BSD-3 header only); code from RESEARCH.md Pattern 4.

**CRITICAL NOTE on `target_pose` / `circ_point`:** These are `geometry_msgs/PoseStamped` and `geometry_msgs/Point` ROS2 C-extension types. To store them in a frozen Pydantic model, use `model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)` (RESEARCH.md Pitfall 7). Phase 2 stub execution does not inspect these fields, but they must flow through the DTO so Phase 3 can use them.

**Full file pattern** (RESEARCH.md Pattern 4; D-15):
```python
# [BSD-3 header]
"""Pydantic DTO mirroring the TrajectoryPath ROS2 message."""

from geometry_msgs.msg import Point, PoseStamped
from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.msg import TrajectoryPath


class TrajectoryPathDTO(BaseModel):
    """Immutable DTO mirroring TrajectoryPath.msg fields with Pydantic validation."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    path_id: str = Field(description='UUID4 path identifier, must be non-empty')
    motion_type: MotionTypeEnum = Field(description='Motion type: LIN, PTP, or CIRC')
    target_pose: PoseStamped = Field(description='Target end-effector pose in planning frame')
    blend_radius: float = Field(default=0.0, description='Blend radius in metres; negative → 0.0')
    cartesian_speed: float = Field(default=0.0, description='End-effector speed in m/s')
    acceleration: float = Field(default=0.0, description='End-effector acceleration in m/s²')
    tool_frame: str = Field(default='', description='Tool frame override; empty string → tool0')
    circ_type: str = Field(default='', description='CIRC point interpretation: interim or center')
    circ_point: Point = Field(
        default_factory=Point,
        description='Auxiliary point for CIRC motion; ignored for LIN/PTP'
    )

    @field_validator('path_id')
    @classmethod
    def validate_path_id_non_empty(cls, v: str) -> str:
        if not v:
            raise ValueError('path_id must be non-empty')
        return v

    @field_validator('blend_radius', mode='before')
    @classmethod
    def normalize_negative_blend_radius(cls, v: float) -> float:
        return 0.0 if float(v) < 0 else float(v)

    @classmethod
    def from_ros_msg(cls, msg: TrajectoryPath) -> 'TrajectoryPathDTO':
        """Construct a DTO from a TrajectoryPath ROS2 message."""
        return cls(
            path_id=msg.path_id,
            motion_type=MotionTypeEnum(msg.motion_type),
            target_pose=msg.target_pose,
            blend_radius=msg.blend_radius,
            cartesian_speed=msg.cartesian_speed,
            acceleration=msg.acceleration,
            tool_frame=msg.tool_frame,
            circ_type=msg.circ_type,
            circ_point=msg.circ_point,
        )
```

---

### `movement_controller/models/trajectory_goal_dto.py` (model/DTO, transform)

**Analog:** `movement_controller/models/__init__.py` (BSD-3 header only); code from RESEARCH.md Pattern 4 + D-15.

**Full file pattern** (D-15; RESEARCH.md Pattern 4):
```python
# [BSD-3 header]
"""Pydantic DTO wrapping the ExecuteTrajectory goal."""

from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class TrajectoryGoalDTO(BaseModel):
    """Immutable DTO representing a validated ExecuteTrajectory goal."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    paths: list[TrajectoryPathDTO] = Field(description='Ordered list of trajectory paths; must be non-empty')

    @field_validator('paths')
    @classmethod
    def validate_paths_non_empty(cls, v: list[TrajectoryPathDTO]) -> list[TrajectoryPathDTO]:
        if not v:
            raise ValueError('paths list must be non-empty')
        return v
```

---

### `movement_controller/utils/trajectory_grouper.py` (utility, transform)

**Analog:** `movement_controller/utils/__init__.py` (BSD-3 header only); code from RESEARCH.md Pattern 6 + D-07–D-10.

**Full file pattern** (RESEARCH.md Pattern 6; D-07, D-08, D-09, D-10):
```python
# [BSD-3 header]
"""Trajectory grouper utility: groups paths by blend radius for execution."""

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class TrajectoryGrouper:
    """Groups trajectory paths into blended execution groups.

    A new group starts at the first path and whenever a path has blend_radius <= 0.
    A path with blend_radius > 0 (and not the first) merges into the current group.
    Negative blend_radius values are expected to have been normalized to 0.0 by
    TrajectoryPathDTO before reaching this class.
    """

    @staticmethod
    def group(paths: list[TrajectoryPathDTO]) -> list[list[TrajectoryPathDTO]]:
        """Group paths into blended execution groups.

        Args:
            paths: Ordered list of trajectory path DTOs. Must be non-empty.
                   All path_ids must be non-empty and unique.

        Returns:
            List of groups; each group is a list of one or more TrajectoryPathDTO.

        Raises:
            ValueError: If paths is empty, any path_id is empty, or path_ids are not unique.
        """
        if not paths:
            raise ValueError('paths list must not be empty')

        seen_ids: set[str] = set()
        for path in paths:
            if not path.path_id:
                raise ValueError('path_id must be non-empty')
            if path.path_id in seen_ids:
                raise ValueError(f'Duplicate path_id: {path.path_id!r}')
            seen_ids.add(path.path_id)

        groups: list[list[TrajectoryPathDTO]] = []
        for i, path in enumerate(paths):
            if i == 0 or path.blend_radius <= 0:
                groups.append([path])
            else:
                groups[-1].append(path)
        return groups
```

**Grouper acceptance test case** (D-07 example, used as unit test):
- Input: `[t0(br=0.5), t1(br=0), t2(br=0), t3(br=0.3), t4(br=0.3), t5(br=0.3), t6(br=0)]`
- Expected: 4 groups: `[[t0], [t1], [t2, t3, t4, t5], [t6]]` → 8 feedback messages total

---

### `tests/unit/test_enums_and_dtos.py` (test, request-response)

**Analog:** `tests/unit/test_imports.py` (exact structure match)

**Header + structure pattern** (`tests/unit/test_imports.py` lines 1–28 + overall structure):
```python
# [BSD-3 header]
"""Unit tests for MotionTypeEnum, FeedbackStatusEnum, TrajectoryPathDTO, TrajectoryGoalDTO."""


def test_motion_type_enum_values():
    from movement_controller.enums.motion_type_enum import MotionTypeEnum
    assert MotionTypeEnum.LIN == "LIN"
    assert MotionTypeEnum.PTP == "PTP"
    assert MotionTypeEnum.CIRC == "CIRC"


def test_feedback_status_enum_values():
    from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
    assert FeedbackStatusEnum.EXECUTING == "executing"
    assert FeedbackStatusEnum.COMPLETED == "completed"


def test_trajectory_path_dto_rejects_invalid_motion_type():
    # D-03: motion_type must be LIN, PTP, or CIRC
    import pytest
    from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
    from geometry_msgs.msg import PoseStamped
    with pytest.raises((ValueError, Exception)):
        TrajectoryPathDTO(path_id='abc', motion_type='INVALID', target_pose=PoseStamped())


def test_trajectory_path_dto_rejects_empty_path_id():
    # D-03: path_id must be non-empty
    import pytest
    from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
    from geometry_msgs.msg import PoseStamped
    with pytest.raises((ValueError, Exception)):
        TrajectoryPathDTO(path_id='', motion_type='LIN', target_pose=PoseStamped())


def test_trajectory_path_dto_normalizes_negative_blend_radius():
    # D-07, D-10: negative blend_radius silently normalized to 0.0
    from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
    from geometry_msgs.msg import PoseStamped
    dto = TrajectoryPathDTO(path_id='p1', motion_type='LIN', target_pose=PoseStamped(), blend_radius=-1.5)
    assert dto.blend_radius == 0.0


def test_trajectory_goal_dto_rejects_empty_paths():
    import pytest
    from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
    with pytest.raises((ValueError, Exception)):
        TrajectoryGoalDTO(paths=[])
```

---

### `tests/unit/test_trajectory_grouper.py` (test, request-response)

**Analog:** `tests/unit/test_imports.py` (exact structure match)

**Structure + canonical acceptance test pattern** (D-07 grouper example):
```python
# [BSD-3 header]
"""Unit tests for TrajectoryGrouper."""

import pytest


def _make_path(path_id: str, blend_radius: float = 0.0):
    """Helper: construct a minimal TrajectoryPathDTO."""
    from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
    from geometry_msgs.msg import PoseStamped
    return TrajectoryPathDTO(path_id=path_id, motion_type='LIN',
                             target_pose=PoseStamped(), blend_radius=blend_radius)


def test_grouper_single_path():
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    groups = TrajectoryGrouper.group([_make_path('p0', 0.0)])
    assert groups == [[_make_path('p0', 0.0)]]


def test_grouper_all_no_blend():
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    paths = [_make_path('p0'), _make_path('p1'), _make_path('p2')]
    groups = TrajectoryGrouper.group(paths)
    assert len(groups) == 3


def test_grouper_canonical_d07_example():
    # D-07: [t0(br=0.5), t1(br=0), t2(br=0), t3(br=0.3), t4(br=0.3), t5(br=0.3), t6(br=0)]
    # Expected: [[t0], [t1], [t2, t3, t4, t5], [t6]]
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    paths = [
        _make_path('t0', 0.5),
        _make_path('t1', 0.0),
        _make_path('t2', 0.0),
        _make_path('t3', 0.3),
        _make_path('t4', 0.3),
        _make_path('t5', 0.3),
        _make_path('t6', 0.0),
    ]
    groups = TrajectoryGrouper.group(paths)
    assert len(groups) == 4
    assert [p.path_id for p in groups[0]] == ['t0']
    assert [p.path_id for p in groups[1]] == ['t1']
    assert [p.path_id for p in groups[2]] == ['t2', 't3', 't4', 't5']
    assert [p.path_id for p in groups[3]] == ['t6']


def test_grouper_raises_on_empty():
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    with pytest.raises(ValueError):
        TrajectoryGrouper.group([])


def test_grouper_raises_on_duplicate_path_id():
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    with pytest.raises(ValueError):
        TrajectoryGrouper.group([_make_path('p0'), _make_path('p0')])


def test_grouper_first_path_blend_radius_ignored():
    # D-07: first path always starts a new group regardless of blend_radius
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    paths = [_make_path('p0', 0.5), _make_path('p1', 0.0)]
    groups = TrajectoryGrouper.group(paths)
    # p0 starts its own group; p1 blend_radius=0 starts a new group
    assert len(groups) == 2


def test_grouper_negative_blend_radius_treated_as_zero():
    # D-07: negative blend_radius == 0; normalized by DTO before reaching grouper
    from movement_controller.utils.trajectory_grouper import TrajectoryGrouper
    # blend_radius=-1 is normalized to 0.0 by TrajectoryPathDTO validator
    paths = [_make_path('p0', 0.0), _make_path('p1', -1.0)]
    groups = TrajectoryGrouper.group(paths)
    # normalized blend_radius=0.0 on p1 → new group
    assert len(groups) == 2
```

---

### `tests/unit/test_ur_movement_controller.py` (test, event-driven)

**Analog:** `tests/unit/test_imports.py` (header + structure); node testing via `unittest.mock`.

**Key mocking strategy** (D-16; copilot-instructions.md §Testing):
- `URMovementController` calls `ActionServer(...)` in `on_configure`. In unit tests, patch `rclpy.action.ActionServer` before instantiating the node.
- `rclpy` requires init; use `rclpy.init()` / `rclpy.shutdown()` in a module-level fixture or per-test setup.
- Lifecycle transitions invoked directly: `node.trigger_configure(...)` → tests check return and log output.
- `_state_machine.current_state` is internal state; set it indirectly by triggering transitions, OR mock it for isolated goal_callback tests.

**Structure pattern**:
```python
# [BSD-3 header]
"""Unit tests for URMovementController lifecycle and action server behaviour."""

import threading
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from lifecycle_msgs.msg import State
from rclpy.action import GoalResponse


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_goal_rejected_when_not_active(ros_context):
    """D-12: goal rejected if node not in PRIMARY_STATE_ACTIVE."""
    with patch('rclpy.action.ActionServer'):
        from movement_controller.ur_movement_controller import URMovementController
        node = URMovementController()
        node.trigger_configure()
        # Node is now INACTIVE, not ACTIVE
        goal = MagicMock()
        goal.paths = [MagicMock(path_id='p1', motion_type='LIN')]
        result = node._goal_callback(goal)
        assert result == GoalResponse.REJECT
        node.destroy_node()


def test_goal_rejected_when_executing(ros_context):
    """D-12, D-13: goal rejected if _is_executing is True."""
    with patch('rclpy.action.ActionServer'):
        from movement_controller.ur_movement_controller import URMovementController
        node = URMovementController()
        node.trigger_configure()
        node.trigger_activate()
        node._is_executing = True
        goal = MagicMock()
        goal.paths = [MagicMock(path_id='p1', motion_type='LIN')]
        result = node._goal_callback(goal)
        assert result == GoalResponse.REJECT
        node.destroy_node()


def test_feedback_sequence_correct(ros_context):
    """D-06: executing→completed pairs sent per group; trajectory_paths_completed echoes all."""
    # ... mock goal_handle, assert publish_feedback calls in order
    pass  # planner fills implementation detail


def test_lifecycle_configure_activate_deactivate_cleanup(ros_context):
    """ACT-05: all lifecycle transitions return SUCCESS."""
    with patch('rclpy.action.ActionServer'):
        from movement_controller.ur_movement_controller import URMovementController
        node = URMovementController()
        assert node.trigger_configure().state.label == 'inactive'
        assert node.trigger_activate().state.label == 'active'
        assert node.trigger_deactivate().state.label == 'inactive'
        assert node.trigger_cleanup().state.label == 'unconfigured'
        node.destroy_node()
```

---

### `CMakeLists.txt` *(modified)* (config)

**Analog:** `CMakeLists.txt` lines 77–91 (exact `ament_add_pytest_test` block for `test_imports`)

**Existing pattern to copy** (`CMakeLists.txt` lines 77–91):
```cmake
  ament_add_pytest_test(test_imports
    "tests/unit/test_imports.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
```

**Three new blocks to add** immediately after the `test_imports` block (RESEARCH.md Pattern 8):
```cmake
  ament_add_pytest_test(test_enums_and_dtos
    "tests/unit/test_enums_and_dtos.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(test_trajectory_grouper
    "tests/unit/test_trajectory_grouper.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(test_ur_movement_controller
    "tests/unit/test_ur_movement_controller.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
```

**⚠️ Rebuild required** after CMakeLists.txt changes (RESEARCH.md Pitfall 2): `colcon build --symlink-install`.

---

### `setup.py` *(modified)* (config)

**Analog:** `setup.py` lines 51–56 (existing `console_scripts` stub)

**Existing stub to replace** (`setup.py` lines 51–56):
```python
    entry_points={
        'console_scripts': [
            # Add node entry points here, e.g.:
            # 'node_name = movement_controller.node_module:main',
        ],
```

**New pattern** (RESEARCH.md §Recommended Project Structure; copilot-instructions.md convention):
```python
    entry_points={
        'console_scripts': [
            'ur_movement_controller = movement_controller.ur_movement_controller:main',
        ],
```

---

## Shared Patterns

### BSD-3-Clause License Header
**Source:** All existing Python files (e.g., `movement_controller/__init__.py` lines 1–28)
**Apply to:** All 7 new Python source files — `ur_movement_controller.py`, `motion_type_enum.py`, `feedback_status_enum.py`, `trajectory_path_dto.py`, `trajectory_goal_dto.py`, `trajectory_grouper.py`, and all 3 test files.
```python
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
# [... full 28-line header as in movement_controller/__init__.py ...]
```

### Error Handling at Node Boundaries
**Source:** RESEARCH.md Pattern 3; copilot-instructions.md §Error Handling
**Apply to:** `ur_movement_controller.py` `_execute_callback`
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

### `TransitionCallbackReturn.SUCCESS` from all lifecycle callbacks
**Source:** RESEARCH.md Anti-Patterns / Pitfall 3
**Apply to:** `ur_movement_controller.py` — all four lifecycle callbacks
- Never return `True`, `None`, or an integer — always return `TransitionCallbackReturn.SUCCESS` or `TransitionCallbackReturn.FAILURE`.

### Pydantic v2 `@field_validator` with `@classmethod`
**Source:** RESEARCH.md Pattern 4 / Pitfall 5
**Apply to:** `trajectory_path_dto.py`, `trajectory_goal_dto.py`
```python
    @field_validator('field_name', mode='before')
    @classmethod
    def validator_name(cls, v):
        ...
```

### Import style in tests: inline imports per test function
**Source:** `tests/unit/test_imports.py` lines 32–34 (imports inside functions)
**Apply to:** All 3 new test files — keep module-level imports minimal; do ROS2/package imports inside test functions to avoid import-time side effects.

---

## No Analog Found

All production code files are greenfield — no existing LifecycleNode, Pydantic model, or utility class exists in the codebase yet. The RESEARCH.md patterns (which are verified against the live devcontainer) serve as the authoritative source for these files.

| File | Role | Data Flow | Reason |
|------|------|-----------|--------|
| `movement_controller/ur_movement_controller.py` | node/controller | event-driven | First LifecycleNode in codebase; pattern from RESEARCH.md Patterns 1–3, 7 |
| `movement_controller/enums/motion_type_enum.py` | enum | transform | First enum in codebase; pattern from RESEARCH.md Pattern 5 |
| `movement_controller/enums/feedback_status_enum.py` | enum | transform | First enum in codebase; pattern from RESEARCH.md Pattern 5 |
| `movement_controller/models/trajectory_path_dto.py` | model/DTO | transform | First Pydantic model in codebase; pattern from RESEARCH.md Pattern 4 |
| `movement_controller/models/trajectory_goal_dto.py` | model/DTO | transform | First Pydantic model in codebase; pattern from RESEARCH.md Pattern 4 |
| `movement_controller/utils/trajectory_grouper.py` | utility | transform | First utility class in codebase; pattern from RESEARCH.md Pattern 6 |

---

## Critical Implementation Warnings

1. **`self.get_current_state()` does NOT exist** in rclpy Jazzy → use `self._state_machine.current_state[0]` (RESEARCH.md Pitfall 1).
2. **`ActionServer` must be created in `on_configure`**, not `__init__` — parameters not declared yet at `__init__` time (RESEARCH.md Pitfall 4).
3. **`_is_executing` cleared in `finally` block** — never skip `try/finally` pattern (RESEARCH.md Pitfall/Anti-Pattern).
4. **CMakeLists.txt requires `colcon build --symlink-install` rebuild** after adding new `ament_add_pytest_test` entries — pytest `--import-mode=importlib` required on all entries (RESEARCH.md Pitfall 2).
5. **`model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)`** required on `TrajectoryPathDTO` because `PoseStamped` and `Point` are C-extension types (RESEARCH.md Pitfall 7).

---

## Metadata

**Analog search scope:** `src/movement_controller/` (all subdirectories)
**Files scanned:** 8 existing Phase 1 files (`__init__.py`, `models/__init__.py`, `enums/__init__.py`, `utils/__init__.py`, `services/__init__.py`, `tests/unit/test_imports.py`, `CMakeLists.txt`, `setup.py`)
**Pattern extraction date:** 2026-05-27
