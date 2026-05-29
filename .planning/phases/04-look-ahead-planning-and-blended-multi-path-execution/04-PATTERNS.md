# Phase 4: Look-Ahead Planning & Blended Multi-Path Execution — Pattern Map

**Mapped:** 2026-05-29
**Files analyzed:** 5
**Analogs found:** 5 / 5

---

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|-------------------|------|-----------|----------------|---------------|
| `movement_controller/services/pilz_planner_service.py` | service | event-driven (bg thread + queue) | itself (current `plan()` method) | exact — same class, expanded |
| `movement_controller/ur_movement_controller.py` | controller | event-driven (action server) | itself (current `_execute_callback`) | exact — same class, updated |
| `movement_controller/models/plan_result_dto.py` | model | transform | itself + `trajectory_path_dto.py` | exact — same file, 2 fields added |
| `tests/unit/test_pilz_planner_service.py` | test | request-response | itself (current unit test file) | exact — same file, new test cases |
| `tests/integration/test_moveit_execution_integration.py` | test | event-driven | itself (current integration test file) | exact — same file, new test cases |

---

## Pattern Assignments

---

### `movement_controller/services/pilz_planner_service.py` (service, event-driven)

**Analog:** [movement_controller/services/pilz_planner_service.py](../../../src/movement_controller/movement_controller/services/pilz_planner_service.py) — current implementation

**Imports pattern** (lines 1–40):
```python
# BSD-3-Clause license header (copy verbatim from existing file)
"""PilzPlannerService — ..."""

from __future__ import annotations

import queue
import threading
import time

from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import robotStateToRobotStateMsg
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MoveItErrorCodes, MotionPlanRequest,
    MotionSequenceItem, MotionSequenceRequest, PositionConstraint,
    OrientationConstraint, RobotState,
)
from moveit_msgs.srv import GetMotionSequence
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import rclpy

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
```

**Constructor expansion pattern** — add `node` kwarg, create service client:
```python
class PilzPlannerService:
    def __init__(self, moveit: MoveItPy, moveit_group_name: str, node) -> None:
        self._moveit = moveit
        self._group_name = moveit_group_name
        self._node = node
        self._planning_component: PlanningComponent = self._moveit.get_planning_component(moveit_group_name)
        # Phase 4: service client for sequence planning (/plan_sequence_path)
        self._plan_seq_client = node.create_client(GetMotionSequence, '/plan_sequence_path')
        # Phase 4: threading state (initialised fresh per plan_all() call — D-04)
        self._cancel_event: threading.Event | None = None
        self._plan_queue: queue.Queue | None = None
        self._planning_thread: threading.Thread | None = None
```

**`plan_all()` pattern** — fresh thread + queue per goal (D-04):
```python
def plan_all(self, groups: list[list[TrajectoryPathDTO]]) -> None:
    """Start fresh background planning thread and queue (D-04)."""
    self._cancel_event = threading.Event()
    self._plan_queue = queue.Queue()
    self._planning_thread = threading.Thread(
        target=self._planning_loop, args=(groups,), daemon=True
    )
    self._planning_thread.start()
```

**`_planning_loop()` pattern** — sequential group planning with state propagation (D-08):
```python
def _planning_loop(self, groups: list[list[TrajectoryPathDTO]]) -> None:
    """Background thread: plan groups sequentially, propagate end-state (D-08)."""
    # Get current state once at thread start (Pitfall 7: no per-group scene monitor access)
    with self._moveit.get_planning_scene_monitor().read_only() as scene:
        current_moveit_state = scene.current_state
    last_predicted_state: RobotState = robotStateToRobotStateMsg(current_moveit_state)

    for group in groups:
        if self._cancel_event.is_set():
            break

        result = self._plan_group_sequence(group, last_predicted_state)

        if self._cancel_event.is_set():
            break

        if result is None:
            # Planning failed — terminate iterator via sentinel
            self._plan_queue.put(StopIteration)
            return

        # Propagate end-state for next group (D-08)
        last_predicted_state = self._extract_end_state(result.trajectories)
        self._plan_queue.put(result)

    self._plan_queue.put(StopIteration)
```

**`_plan_group_sequence()` pattern** — build MotionSequenceRequest, call service (Finding 2, D-07):
```python
def _plan_group_sequence(
    self, group: list[TrajectoryPathDTO], start_state_msg: RobotState
) -> PlanResultDTO | None:
    """Call /plan_sequence_path service. Returns None on failure."""
    seq_req = MotionSequenceRequest()

    for i, path_dto in enumerate(group):
        item = MotionSequenceItem()
        # Pitfall 1: last item's blend_radius MUST be 0.0
        item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
        item.req.group_name = self._group_name
        item.req.pipeline_id = 'pilz_industrial_motion_planner'
        item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', 'CIRC'
        item.req.allowed_planning_time = 5.0
        item.req.max_velocity_scaling_factor = 0.1
        item.req.max_acceleration_scaling_factor = 0.1
        item.req.goal_constraints = [
            self._build_pose_goal_constraints(path_dto.tool_frame or 'tool0', path_dto.target_pose)
        ]
        if i == 0:
            item.req.start_state = start_state_msg  # only items[0] — Finding 2
        if path_dto.motion_type == MotionTypeEnum.CIRC:
            item.req.path_constraints = self._build_circ_constraints(path_dto)
        seq_req.items.append(item)

    request = GetMotionSequence.Request()
    request.request = seq_req
    future = self._plan_seq_client.call_async(request)

    # Poll until done — executor in main thread resolves Future (Finding 1)
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

**`_extract_end_state()` pattern** — state propagation (Finding 5):
```python
@staticmethod
def _extract_end_state(trajectories: list) -> RobotState:
    """Extract final joint state from last trajectory for state propagation (D-08)."""
    last_trajectory = trajectories[-1]
    jt = last_trajectory.joint_trajectory
    last_point = jt.points[-1]
    predicted = RobotState()
    predicted.joint_state.name = list(jt.joint_names)
    predicted.joint_state.position = list(last_point.positions)
    predicted.joint_state.velocity = [0.0] * len(jt.joint_names)
    return predicted
```

**`iterate_planned_trajectories()` generator pattern** — blocking dequeue, StopIteration sentinel (D-05):
```python
def iterate_planned_trajectories(self):
    """Generator: yields PlanResultDTO; blocks on empty queue; terminates on sentinel (D-05)."""
    while True:
        item = self._plan_queue.get()  # blocking
        if item is StopIteration:
            return
        yield item
```

**`cancel()` pattern** — thread-safe drain + sentinel (D-09, Finding 9):
```python
def cancel(self) -> None:
    """Thread-safe cancel: set event, drain queue atomically, push sentinel (D-09)."""
    if self._cancel_event is not None:
        self._cancel_event.set()
    if self._plan_queue is not None:
        with self._plan_queue.mutex:
            self._plan_queue.queue.clear()
        self._plan_queue.put(StopIteration)
```

**`_build_pose_goal_constraints()` pattern** — goal constraints from PoseStamped (Finding 6):
```python
@staticmethod
def _build_pose_goal_constraints(
    link_name: str,
    pose_stamped,
    position_tolerance: float = 0.0001,
    orientation_tolerance: float = 0.001,
) -> Constraints:
    """Build MoveIt2 goal Constraints from PoseStamped (replaces C++ constructGoalConstraints)."""
    constraints = Constraints()

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

**Existing `_build_circ_constraints()` pattern** — REUSE UNCHANGED from lines 110–145 of current file:
```python
# Existing static method — copy unchanged into Phase 4 version.
# constraints.name = path_dto.circ_type.value  ('interim' or 'center')
# PositionConstraint with SolidPrimitive.SPHERE + BoundingVolume for circ_point
```

---

### `movement_controller/ur_movement_controller.py` (controller, event-driven)

**Analog:** [movement_controller/ur_movement_controller.py](../../../src/movement_controller/movement_controller/ur_movement_controller.py) — current implementation

**Imports to add** (extend existing import block, lines 29–42):
```python
# Add to existing imports:
from rclpy.action import ActionServer, GoalResponse, CancelResponse  # add CancelResponse
from moveit.core.robot_trajectory import RobotTrajectory
```

**`on_configure()` change — add `cancel_callback` and pass `node=self` to service** (lines 89–115):
```python
# CHANGE 1: Update PilzPlannerService constructor call (Pitfall 6)
self._planner_service = PilzPlannerService(self._moveit, moveit_group_name, node=self)

# CHANGE 2: Wait for /plan_sequence_path availability (Pitfall 4)
if not self._planner_service._plan_seq_client.wait_for_service(timeout_sec=timeout):
    self.get_logger().error('/plan_sequence_path not available — PILZ sequence capability loaded?')
    return TransitionCallbackReturn.FAILURE

# CHANGE 3: Add cancel_callback to ActionServer
self._action_server = ActionServer(
    self,
    ExecuteTrajectory,
    "movement_controller/execute_trajectory",
    execute_callback=self._execute_callback,
    goal_callback=self._goal_callback,
    cancel_callback=self._cancel_callback,   # ← new (D-10)
    callback_group=ReentrantCallbackGroup(),
)
```

**`_cancel_callback()` pattern** — non-blocking, return immediately (D-10, Finding 8):
```python
def _cancel_callback(self, cancel_request) -> CancelResponse:
    """Non-blocking cancel: signal planner, return ACCEPT immediately (D-10)."""
    if self._planner_service is not None:
        self._planner_service.cancel()
    return CancelResponse.ACCEPT
```

**`_execute_callback()` replacement pattern** — generator loop with group-level feedback (D-01, Architecture Pattern 3):
```python
async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecuteTrajectory.Result:
    try:
        goal_dto = TrajectoryGoalDTO.from_ros_msg(goal_handle.request)
        groups = TrajectoryGrouper.group(goal_dto.paths)
        completed_ids: list[str] = []

        if self._planner_service is None or self._moveit is None:
            err = 'Planner service or MoveItPy not initialised'
            self.get_logger().error(err)
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = err
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result

        robot_model = self._moveit.get_robot_model()
        tem = self._moveit.get_trajectory_execution_manager()

        self._planner_service.plan_all(groups)

        for plan_dto in self._planner_service.iterate_planned_trajectories():
            # Cancel check (Finding 8)
            if goal_handle.is_cancel_requested:
                self._planner_service.cancel()
                result = ExecuteTrajectory.Result()
                result.success = False
                result.error_message = 'Goal cancelled'
                result.trajectory_paths_completed = completed_ids
                try:
                    goal_handle.canceled()
                except Exception:
                    pass
                return result

            if not self._is_active:
                self.get_logger().warn('Execution halted: node deactivated mid-trajectory')
                result = ExecuteTrajectory.Result()
                result.success = False
                result.error_message = 'Node deactivated during execution'
                try:
                    goal_handle.abort()
                except Exception:
                    pass
                return result

            if not plan_dto.success:
                err = plan_dto.error_message or f'Planning failed for group {plan_dto.path_ids}'
                self.get_logger().error(err)
                result = ExecuteTrajectory.Result()
                result.success = False
                result.error_message = err
                result.trajectory_paths_completed = completed_ids
                try:
                    goal_handle.abort()
                except Exception:
                    pass
                return result

            # Group-level 'executing' feedback (D-01)
            fb = ExecuteTrajectory.Feedback()
            fb.status = FeedbackStatusEnum.EXECUTING.value
            fb.trajectory_path_ids = plan_dto.path_ids
            goal_handle.publish_feedback(fb)

            # Execute blended trajectory segments (Finding 7)
            with self._moveit.get_planning_scene_monitor().read_only() as scene:
                ref_state = scene.current_state
            for ros_traj_msg in plan_dto.trajectories:
                traj = RobotTrajectory(robot_model)
                traj.set_robot_trajectory_msg(ref_state, ros_traj_msg)
                tem.push(traj)
            exec_ok = tem.execute_and_wait()

            if not exec_ok:
                err = f'Execution failed for group {plan_dto.path_ids}'
                self.get_logger().error(err)
                result = ExecuteTrajectory.Result()
                result.success = False
                result.error_message = err
                result.trajectory_paths_completed = completed_ids
                try:
                    goal_handle.abort()
                except Exception:
                    pass
                return result

            completed_ids.extend(plan_dto.path_ids)

            # Group-level 'completed' feedback (D-01)
            fb2 = ExecuteTrajectory.Feedback()
            fb2.status = FeedbackStatusEnum.COMPLETED.value
            fb2.trajectory_path_ids = plan_dto.path_ids
            goal_handle.publish_feedback(fb2)

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
        try:
            goal_handle.abort()
        except Exception:
            pass
        return result

    finally:
        with self._executing_lock:
            self._is_executing = False
```

---

### `movement_controller/models/plan_result_dto.py` (model, transform)

**Analog:** [movement_controller/models/plan_result_dto.py](../../../src/movement_controller/movement_controller/models/plan_result_dto.py) — current file (58 lines)

**New fields to add** — D-06 (`path_ids`, `blended`) + Phase 4 `trajectories` list:
```python
# Copy entire existing file, then ADD these fields alongside existing ones:

from moveit_msgs.msg import RobotTrajectory as RosRobotTrajectory  # type alias for clarity

class PlanResultDTO(BaseModel):
    """Internal immutable result returned by PilzPlannerService."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    success: bool = Field(description='True if PILZ planning succeeded')
    trajectory: Any = Field(
        default=None,
        description='RobotTrajectory from MoveItPy (Phase 3 single-path); None when using trajectories list',
    )
    trajectories: list[Any] = Field(
        default_factory=list,
        description='List of moveit_msgs/RobotTrajectory from MotionSequenceResponse (Phase 4+)',
    )
    path_ids: list[str] = Field(
        default_factory=list,
        description='Path IDs this result covers; used by controller for feedback (D-06)',
    )
    blended: bool = Field(
        default=False,
        description='True if group has >1 path (blend group); False for single-path groups (D-06)',
    )
    error_message: str = Field(
        default='',
        description='Human-readable error; empty on success',
    )
```

**Pattern note:** `default_factory=list` is the Pydantic v2 pattern for mutable defaults (copy from `TrajectoryPathDTO.circ_point` which uses `default_factory=Point`). `frozen=True` is preserved — all new fields use defaults or factory so they work with frozen models.

---

### `tests/unit/test_pilz_planner_service.py` (test, request-response)

**Analog:** [tests/unit/test_pilz_planner_service.py](../../../src/movement_controller/tests/unit/test_pilz_planner_service.py) — current file

**License header:** Copy BSD-3-Clause header verbatim from existing file (lines 1–28).

**New fixture pattern** — mock service client + `GetMotionSequence` response:
```python
# Add to existing fixtures:
@pytest.fixture
def mock_node():
    """Mock rclpy node with create_client."""
    node = MagicMock()
    node.create_client.return_value = MagicMock()
    return node

@pytest.fixture
def mock_seq_client():
    """Mock GetMotionSequence service client with successful response."""
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory as RosTraj
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    jt = JointTrajectory()
    jt.joint_names = ['joint_1', 'joint_2']
    pt = JointTrajectoryPoint()
    pt.positions = [0.1, 0.2]
    jt.points = [pt]

    ros_traj = RosTraj()
    ros_traj.joint_trajectory = jt

    response_inner = MagicMock()
    response_inner.error_code.val = MoveItErrorCodes.SUCCESS
    response_inner.planned_trajectories = [ros_traj]

    response_outer = MagicMock()
    response_outer.response = response_inner

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = response_outer

    client = MagicMock()
    client.call_async.return_value = future
    client.wait_for_service.return_value = True
    return client

@pytest.fixture
def service_v4(mock_moveit, mock_planning_component, mock_node, mock_seq_client):
    """PilzPlannerService (Phase 4) with node injected and mocked service client."""
    mock_moveit.get_planning_component.return_value = mock_planning_component
    mock_node.create_client.return_value = mock_seq_client
    svc = PilzPlannerService(mock_moveit, 'ur_manipulator', node=mock_node)
    return svc
```

**New test case pattern** — `plan_all` + `iterate_planned_trajectories`:
```python
def test_plan_all_yields_plan_result_dto(service_v4, mock_moveit):
    """plan_all() + iterate_planned_trajectories() yields one PlanResultDTO per group."""
    path = _make_path_dto(motion_type=MotionTypeEnum.LIN)

    # Mock scene monitor for start state extraction
    mock_scene = MagicMock()
    mock_state = MagicMock()
    mock_state.joint_state.name = ['j1']
    mock_state.joint_state.position = [0.0]
    mock_scene.__enter__ = MagicMock(return_value=mock_scene)
    mock_scene.__exit__ = MagicMock(return_value=False)
    mock_scene.current_state = MagicMock()
    mock_moveit.get_planning_scene_monitor.return_value.read_only.return_value = mock_scene

    # Patch robotStateToRobotStateMsg
    with patch('movement_controller.services.pilz_planner_service.robotStateToRobotStateMsg') as mock_rsm:
        mock_rsm.return_value = MagicMock()
        service_v4.plan_all([[path]])
        results = list(service_v4.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].path_ids == [path.path_id]
    assert results[0].blended is False  # single-path group


def test_cancel_terminates_iteration(service_v4):
    """cancel() causes iterate_planned_trajectories() to stop."""
    import threading

    service_v4._cancel_event = threading.Event()
    service_v4._plan_queue = __import__('queue').Queue()

    # Start consumer in background, cancel immediately
    results = []
    def consume():
        for r in service_v4.iterate_planned_trajectories():
            results.append(r)

    t = threading.Thread(target=consume)
    t.start()
    service_v4.cancel()
    t.join(timeout=2.0)

    assert not t.is_alive(), 'iterator did not terminate after cancel()'


def test_plan_group_sequence_sets_last_item_blend_radius_to_zero(service_v4):
    """Last item in a blend group always has blend_radius=0.0 (Pitfall 1)."""
    path_a = _make_path_dto(motion_type=MotionTypeEnum.LIN, blend_radius=0.05)
    path_b = _make_path_dto(motion_type=MotionTypeEnum.LIN, blend_radius=0.05)
    start_state = MagicMock()

    service_v4._plan_group_sequence([path_a, path_b], start_state)

    call_args = service_v4._plan_seq_client.call_async.call_args
    items = call_args.args[0].request.items
    assert items[-1].blend_radius == 0.0
```

**Test structure mirroring** — copy existing region comments (`# region:`, `# endregion:`) and keep test names as `test_<what>_<condition>` (snake_case). Use `monkeypatch` for per-test overrides (see existing `test_execute_trajectory_aborts_on_plan_failure`).

---

### `tests/integration/test_moveit_execution_integration.py` (test, event-driven)

**Analog:** [tests/integration/test_moveit_execution_integration.py](../../../src/movement_controller/tests/integration/test_moveit_execution_integration.py) — current file

**License header:** Copy BSD-3-Clause header verbatim from existing file (lines 1–28).

**New fixture pattern** — Phase 4 node with `plan_all` / `iterate_planned_trajectories` mock:
```python
_UUID2 = '00000000-0000-4000-8000-000000000002'
_UUID3 = '00000000-0000-4000-8000-000000000003'

@pytest.fixture
def node_with_moveit_v4(ros_context):
    """URMovementController with Phase 4 planner service mocked (plan_all / iterate)."""
    mock_moveit = MagicMock()
    mock_moveit.execute.return_value = MagicMock(__bool__=MagicMock(return_value=True))
    mock_moveit.get_robot_model.return_value = MagicMock()
    mock_moveit.get_trajectory_execution_manager.return_value = MagicMock()
    mock_moveit.get_planning_scene_monitor.return_value.read_only.return_value.__enter__ = (
        MagicMock(return_value=MagicMock(current_state=MagicMock()))
    )
    mock_moveit.get_planning_scene_monitor.return_value.read_only.return_value.__exit__ = (
        MagicMock(return_value=False)
    )

    with patch('movement_controller.ur_movement_controller.MoveItPy', return_value=mock_moveit):
        n = URMovementController()

    n._is_active = True
    n._moveit = mock_moveit

    yield n
    n.destroy_node()
```

**Multi-path blended test pattern** — 3 paths in 1 blend group, group-level feedback (D-01, D-02):
```python
def test_execute_blended_3path_success(node_with_moveit_v4):
    """3-path blend group executes with group-level feedback, no per-path stops."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.is_cancel_requested = False
    mock_goal_handle.request.paths = [
        _make_path_msg(_UUID1, 'LIN', blend_radius=0.05),
        _make_path_msg(_UUID2, 'LIN', blend_radius=0.05),
        _make_path_msg(_UUID3, 'LIN', blend_radius=0.0),
    ]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    # Mock planner service: plan_all is a no-op; iterate yields 1 group-level DTO
    from movement_controller.models.plan_result_dto import PlanResultDTO
    from moveit_msgs.msg import RobotTrajectory as RosTraj
    plan_dto = PlanResultDTO(
        success=True,
        trajectories=[RosTraj(), RosTraj(), RosTraj()],
        path_ids=[_UUID1, _UUID2, _UUID3],
        blended=True,
    )
    mock_planner = MagicMock()
    mock_planner.plan_all = MagicMock()
    mock_planner.iterate_planned_trajectories = MagicMock(return_value=iter([plan_dto]))
    node_with_moveit_v4._planner_service = mock_planner

    result = asyncio.run(node_with_moveit_v4._execute_callback(mock_goal_handle))

    assert result.success is True
    assert result.trajectory_paths_completed == [_UUID1, _UUID2, _UUID3]
    # D-01: exactly 2 feedback messages for 1 group (executing + completed)
    assert mock_goal_handle.publish_feedback.call_count == 2

    calls = mock_goal_handle.publish_feedback.call_args_list
    executing_fb = calls[0].args[0]
    completed_fb = calls[1].args[0]
    assert executing_fb.status == 'executing'
    assert set(executing_fb.trajectory_path_ids) == {_UUID1, _UUID2, _UUID3}
    assert completed_fb.status == 'completed'


def test_execute_cancel_mid_trajectory(node_with_moveit_v4):
    """Cancel during execution: iterator exits, completed_ids is partial (D-02)."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.is_cancel_requested = True  # cancel immediately
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'LIN')]

    mock_planner = MagicMock()
    mock_planner.plan_all = MagicMock()
    mock_planner.iterate_planned_trajectories = MagicMock(return_value=iter([
        MagicMock(success=True, trajectories=[], path_ids=[_UUID1], blended=False)
    ]))
    node_with_moveit_v4._planner_service = mock_planner

    result = asyncio.run(node_with_moveit_v4._execute_callback(mock_goal_handle))

    assert result.success is False
    mock_planner.cancel.assert_called_once()
```

---

## Shared Patterns

### BSD-3-Clause License Header
**Source:** All existing source files (e.g., [pilz_planner_service.py lines 1–28](../../../src/movement_controller/movement_controller/services/pilz_planner_service.py))
**Apply to:** All modified files — preserve exact header verbatim.
```python
# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# ... (27 lines total — copy from any existing source file)
```

### Error Handling at Action Server Boundary
**Source:** [ur_movement_controller.py lines 204–253](../../../src/movement_controller/movement_controller/ur_movement_controller.py)
**Apply to:** All new failure branches in `_execute_callback`
```python
# Pattern: log error → build Result → try goal_handle.abort() swallowing exception
self.get_logger().error(f'...: {err}')
result = ExecuteTrajectory.Result()
result.success = False
result.error_message = err
result.trajectory_paths_completed = completed_ids  # partial — D-02
try:
    goal_handle.abort()
except Exception:
    pass  # already in terminal state (client cancelled concurrently)
return result
```

### Pydantic Frozen Model with Mutable Default Fields
**Source:** [trajectory_path_dto.py lines 68–80](../../../src/movement_controller/movement_controller/models/trajectory_path_dto.py)
**Apply to:** `plan_result_dto.py` new `list` fields
```python
# Use default_factory for mutable defaults in frozen models:
path_ids: list[str] = Field(default_factory=list, description='...')
trajectories: list[Any] = Field(default_factory=list, description='...')
```

### Test Fixture: `_make_path_msg` helper
**Source:** [test_moveit_execution_integration.py lines 50–67](../../../src/movement_controller/tests/integration/test_moveit_execution_integration.py)
**Apply to:** All new integration tests that need `TrajectoryPath`-like message mocks
```python
def _make_path_msg(path_id, motion_type='LIN', blend_radius=0.0, circ_type='interim'):
    msg = MagicMock()
    msg.path_id = path_id
    msg.motion_type = motion_type
    msg.blend_radius = blend_radius
    msg.target_pose = PoseStamped()
    msg.circ_point = Point()
    msg.cartesian_speed = 0.0
    msg.acceleration = 0.0
    msg.tool_frame = ''
    msg.circ_type = circ_type
    return msg
```

### `patch_plan_request_params` autouse fixture (unit tests)
**Source:** [test_pilz_planner_service.py lines 66–72](../../../src/movement_controller/tests/unit/test_pilz_planner_service.py)
**Apply to:** New unit tests that exercise `plan()` (Phase 3 path) — not needed for `plan_all` tests (those use the service client mock instead)
```python
@pytest.fixture(autouse=True)
def patch_plan_request_params():
    with patch('movement_controller.services.pilz_planner_service.PlanRequestParameters') as mock_cls:
        yield mock_cls
```

---

## No Analog Found

All 5 files have strong analogs (exact match — same file being expanded). No new files lack analogs.

| New File | Reason No Analog |
|----------|-----------------|
| *(none)* | — |

> **Note on optional `utils/motion_sequence_builder.py`:** RESEARCH.md suggests this as a clean extraction of `_build_pose_goal_constraints`. The CONTEXT.md decisions do not require a separate file — D-07 allows the helper to live as a `@staticmethod` on `PilzPlannerService`. The planner should treat this file as optional; if created, the pattern is: a module-level function (not a class), no imports from the package itself, and no state.

---

## Metadata

**Analog search scope:** `src/movement_controller/movement_controller/`, `src/movement_controller/tests/`
**Files scanned:** 8 (pilz_planner_service.py, ur_movement_controller.py, plan_result_dto.py, trajectory_path_dto.py, test_pilz_planner_service.py, test_moveit_execution_integration.py, trajectory_goal_dto.py, trajectory_grouper.py)
**Pattern extraction date:** 2026-05-29
