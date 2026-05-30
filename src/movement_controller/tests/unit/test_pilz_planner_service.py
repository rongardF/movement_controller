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
"""Unit tests for PilzPlannerService — all MoveItPy dependencies mocked."""

import threading
import time
from unittest.mock import MagicMock, call, patch

import pytest
from geometry_msgs.msg import Point, PoseStamped

from movement_controller.enums.circ_type_enum import CircTypeEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService

_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID2 = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'


def _make_path_dto(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with sensible defaults."""
    defaults: dict = {
        'path_id': _UUID,
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


@pytest.fixture
def mock_plan_result():
    """A truthy plan result with a mock trajectory."""
    mr = MagicMock()
    mr.__bool__ = MagicMock(return_value=True)
    mr.trajectory = MagicMock()
    return mr


@pytest.fixture
def mock_planning_component(mock_plan_result):
    """Mock PlanningComponent whose plan() returns a truthy result by default."""
    pc = MagicMock()
    pc.plan.return_value = mock_plan_result
    return pc


@pytest.fixture
def mock_moveit():
    """Mock MoveItPy instance."""
    return MagicMock()


@pytest.fixture
def mock_node():
    n = MagicMock()
    n.create_client.return_value = MagicMock()
    return n


@pytest.fixture
def service(mock_moveit, mock_planning_component, mock_node):
    """PilzPlannerService with injected mocks.

    Wires mock_moveit.get_planning_component to return mock_planning_component so
    that assertions on mock_planning_component still work after the constructor change
    that now calls get_planning_component(group_name) internally.
    """
    mock_moveit.get_planning_component.return_value = mock_planning_component
    return PilzPlannerService(mock_moveit, 'ur_manipulator', mock_node)


@pytest.fixture(autouse=True)
def patch_plan_request_params():
    """Patch PlanRequestParameters so no real MoveItPy constructor is invoked."""
    with patch(
        'movement_controller.services.pilz_planner_service.PlanRequestParameters'
    ) as mock_cls:
        yield mock_cls


# region: Success-path tests


def test_plan_lin_success(service, mock_planning_component, patch_plan_request_params):
    """LIN path returns PlanResultDTO(success=True) with a non-None trajectory."""
    path = _make_path_dto(motion_type=MotionTypeEnum.LIN)

    result = service.plan(path)

    assert result.success is True
    assert result.trajectory is not None
    mock_planning_component.set_start_state_to_current_state.assert_called_once()


def test_plan_ptp_success(service, mock_planning_component, patch_plan_request_params):
    """PTP path returns success=True; set_goal_state is called with default tool_frame='tool0'."""
    path = _make_path_dto(motion_type=MotionTypeEnum.PTP)

    result = service.plan(path)

    assert result.success is True
    mock_planning_component.set_goal_state.assert_called_once_with(
        pose_stamped_msg=path.target_pose,
        pose_link='tool0',
    )


def test_plan_circ_success(service, mock_planning_component, patch_plan_request_params):
    """CIRC path returns success=True; set_path_constraints called with name='interim' then cleared."""
    path = _make_path_dto(
        motion_type=MotionTypeEnum.CIRC,
        circ_type=CircTypeEnum.INTERIM,
        circ_point=Point(x=0.1, y=0.2, z=0.3),
    )

    result = service.plan(path)

    assert result.success is True

    calls = mock_planning_component.set_path_constraints.call_args_list
    assert len(calls) == 2, f'Expected 2 set_path_constraints calls, got {len(calls)}'

    # First call: constraints with name='interim'
    first_constraints = calls[0].args[0]
    assert first_constraints.name == 'interim', (
        f'Expected constraints.name="interim", got {first_constraints.name!r}'
    )

    # Second call: empty Constraints() to clear
    second_constraints = calls[1].args[0]
    assert second_constraints.name == '', (
        f'Expected empty constraints.name for clear call, got {second_constraints.name!r}'
    )


def test_plan_uses_pilz_pipeline(service, patch_plan_request_params):
    """planning_pipeline is set to 'pilz_industrial_motion_planner'."""
    path = _make_path_dto()

    service.plan(path)

    params = patch_plan_request_params.return_value
    assert params.planning_pipeline == 'pilz_industrial_motion_planner'


def test_plan_sets_goal_pose(service, mock_planning_component, patch_plan_request_params):
    """set_goal_state is called with the path's target_pose and resolved tool_frame."""
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    path = _make_path_dto(target_pose=pose, tool_frame='custom_frame')

    service.plan(path)

    mock_planning_component.set_goal_state.assert_called_once_with(
        pose_stamped_msg=pose,
        pose_link='custom_frame',
    )


def test_plan_uses_default_speed_scaling(service, patch_plan_request_params):
    """max_velocity/acceleration_scaling_factor are hardcoded to 0.1 (Phase 5 defers conversion)."""
    path = _make_path_dto(cartesian_speed=1.5, acceleration=2.0)

    service.plan(path)

    params = patch_plan_request_params.return_value
    assert params.max_velocity_scaling_factor == 0.1
    assert params.max_acceleration_scaling_factor == 0.1


# endregion: Success-path tests
# region: Failure-path tests

def test_plan_returns_failure_when_planning_fails(
    service, mock_planning_component, patch_plan_request_params
):
    """When plan() returns a falsy result, PlanResultDTO(success=False) is returned."""
    failing_result = MagicMock()
    failing_result.__bool__ = MagicMock(return_value=False)
    mock_planning_component.plan.return_value = failing_result

    path = _make_path_dto(motion_type=MotionTypeEnum.LIN)

    result = service.plan(path)

    assert result.success is False
    assert 'LIN' in result.error_message


def test_plan_circ_clears_constraints_on_planning_failure(
    service, mock_planning_component, patch_plan_request_params
):
    """set_path_constraints is called twice even when planning fails (try/finally)."""
    failing_result = MagicMock()
    failing_result.__bool__ = MagicMock(return_value=False)
    mock_planning_component.plan.return_value = failing_result

    path = _make_path_dto(
        motion_type=MotionTypeEnum.CIRC,
        circ_type=CircTypeEnum.CENTER,
        circ_point=Point(x=0.5, y=0.5, z=0.5),
    )

    result = service.plan(path)

    assert result.success is False
    calls = mock_planning_component.set_path_constraints.call_args_list
    assert len(calls) == 2, (
        f'Expected constraints to be set then cleared (2 calls), got {len(calls)}'
    )
    # Second call must be the clear call (empty Constraints)
    second_constraints = calls[1].args[0]
    assert second_constraints.name == ''

# endregion: Failure-path tests


# region: Phase 4 — plan_all / iterate / cancel tests

@pytest.fixture
def patch_robot_state_msg():
    """Patch robotStateToRobotStateMsg for the duration of Phase 4 tests."""
    with patch(
        'movement_controller.services.pilz_planner_service.robotStateToRobotStateMsg',
        return_value=MagicMock()
    ) as p:
        yield p


def _build_service_for_phase4(seq_response=None):
    """Build a PilzPlannerService with mocked /plan_sequence_path service client.

    seq_response: mock response; if None, builds a default success response.
    """
    if seq_response is None:
        mock_future = MagicMock()
        mock_future.done.return_value = True
        resp = MagicMock()
        resp.response.error_code.val = 1  # MoveItErrorCodes.SUCCESS
        mock_traj = MagicMock()
        mock_traj.joint_trajectory.joint_names = ['joint_1']
        mock_traj.joint_trajectory.points = [MagicMock(positions=[0.0])]
        resp.response.planned_trajectories = [mock_traj]
        mock_future.result.return_value = resp
    else:
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = seq_response

    mock_client = MagicMock()
    mock_client.call_async.return_value = mock_future

    mock_node = MagicMock()
    mock_node.create_client.return_value = mock_client

    mock_moveit = MagicMock()
    mock_moveit.get_planning_component.return_value = MagicMock()
    mock_scene_ctx = MagicMock()
    mock_scene_ctx.__enter__ = MagicMock(return_value=MagicMock(current_state=MagicMock()))
    mock_scene_ctx.__exit__ = MagicMock(return_value=False)
    mock_moveit.get_planning_scene_monitor.return_value.read_only.return_value = mock_scene_ctx

    svc = PilzPlannerService(mock_moveit, 'ur_manipulator', mock_node)
    svc._mock_client = mock_client  # expose for inspection
    return svc


def test_plan_all_starts_background_thread(patch_robot_state_msg):
    """plan_all() starts a daemon thread; thread is alive (or completes) after call."""
    svc = _build_service_for_phase4()
    path = _make_path_dto()
    svc.plan_all([[path]])
    # Thread starts; it completes quickly since the mock future is synchronous
    svc._planning_thread.join(timeout=2.0)
    assert svc._planning_thread is not None
    # Drain any items that were pushed
    list(svc.iterate_planned_trajectories())


def test_iterate_yields_single_path_result():
    """iterate_planned_trajectories yields PlanResultDTOs from the queue, then terminates."""
    import queue as queue_module
    svc = _build_service_for_phase4()
    svc._plan_queue = queue_module.Queue()
    svc._cancel_event = __import__('threading').Event()

    dto = PlanResultDTO(success=True, path_ids=[_UUID], blended=False, trajectories=[MagicMock()])
    svc._plan_queue.put(dto)
    svc._plan_queue.put(StopIteration)

    results = list(svc.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].path_ids == [_UUID]
    assert results[0].blended is False
    assert len(results[0].trajectories) == 1


def test_iterate_blended_group_sets_blended_true():
    """Two-path group result with blended=True and both path IDs."""
    import queue as queue_module
    svc = _build_service_for_phase4()
    svc._plan_queue = queue_module.Queue()
    svc._cancel_event = __import__('threading').Event()

    dto = PlanResultDTO(
        success=True,
        path_ids=[_UUID, _UUID2],
        blended=True,
        trajectories=[MagicMock(), MagicMock()],
    )
    svc._plan_queue.put(dto)
    svc._plan_queue.put(StopIteration)

    results = list(svc.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].blended is True
    assert results[0].path_ids == [_UUID, _UUID2]


def test_last_item_blend_radius_forced_to_zero():
    """In a 2-path group, items[-1].blend_radius is 0.0 even when path blend_radius=0.05."""
    # Call _plan_group_sequence directly to test PILZ blend_radius constraint without threading.
    svc = _build_service_for_phase4()

    resp2 = MagicMock()
    resp2.response.error_code.val = 1
    mock_traj1 = MagicMock()
    mock_traj1.joint_trajectory.joint_names = ['j1']
    mock_traj1.joint_trajectory.points = [MagicMock(positions=[0.0])]
    mock_traj2 = MagicMock()
    mock_traj2.joint_trajectory.joint_names = ['j1']
    mock_traj2.joint_trajectory.points = [MagicMock(positions=[0.0])]
    resp2.response.planned_trajectories = [mock_traj1, mock_traj2]

    captured_items = []

    def capture_call_async(request):
        captured_items.extend(request.request.items)
        mock_future = MagicMock()
        mock_future.done.return_value = True
        mock_future.result.return_value = resp2
        return mock_future

    svc._mock_client.call_async.side_effect = capture_call_async

    from moveit_msgs.msg import RobotState
    path1 = _make_path_dto(path_id=_UUID, blend_radius=0.05)
    path2 = _make_path_dto(path_id=_UUID2, blend_radius=0.05)
    svc._cancel_event = __import__('threading').Event()
    svc._plan_group_sequence([path1, path2], RobotState())

    assert len(captured_items) == 2, f'Expected 2 items, got {len(captured_items)}'
    assert captured_items[-1].blend_radius == 0.0, (
        f'Last item blend_radius should be 0.0, got {captured_items[-1].blend_radius}'
    )


def test_cancel_terminates_iterator_cleanly():
    """cancel() while planning is blocked causes iterator to return quickly."""
    # Blocking future — never completes until cancel_event is set
    blocking_future = MagicMock()
    blocking_future.done.return_value = False

    mock_client = MagicMock()
    mock_client.call_async.return_value = blocking_future

    mock_node = MagicMock()
    mock_node.create_client.return_value = mock_client

    mock_moveit = MagicMock()
    mock_moveit.get_planning_component.return_value = MagicMock()
    mock_scene_ctx = MagicMock()
    mock_scene_ctx.__enter__ = MagicMock(return_value=MagicMock(current_state=MagicMock()))
    mock_scene_ctx.__exit__ = MagicMock(return_value=False)
    mock_moveit.get_planning_scene_monitor.return_value.read_only.return_value = mock_scene_ctx

    with patch(
        'movement_controller.services.pilz_planner_service.robotStateToRobotStateMsg',
        return_value=MagicMock()
    ):
        svc = PilzPlannerService(mock_moveit, 'ur_manipulator', mock_node)
        svc.plan_all([[_make_path_dto()]])

    # Cancel after brief delay
    def do_cancel():
        time.sleep(0.05)
        svc.cancel()

    threading.Thread(target=do_cancel, daemon=True).start()

    # Iterator should return within 2 seconds
    done_event = threading.Event()

    def drain():
        list(svc.iterate_planned_trajectories())
        done_event.set()

    threading.Thread(target=drain, daemon=True).start()
    assert done_event.wait(timeout=2.0), 'Iterator did not terminate after cancel() — possible hang'


def test_planning_failure_yields_error_dto(patch_robot_state_msg):
    """Service returning error_code.val != 1 causes PlanResultDTO(success=False) to be yielded."""
    fail_resp = MagicMock()
    fail_resp.response.error_code.val = 99  # not SUCCESS
    svc = _build_service_for_phase4()

    fail_future = MagicMock()
    fail_future.done.return_value = True
    fail_future.result.return_value = fail_resp
    svc._mock_client.call_async.return_value = fail_future

    svc.plan_all([[_make_path_dto()]])
    svc._planning_thread.join(timeout=2.0)
    results = list(svc.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is False
    assert results[0].error_message != ''


def test_plan_all_creates_fresh_queue_per_call(patch_robot_state_msg):
    """plan_all() creates a new queue.Queue on each call (D-04: fresh per goal invocation)."""
    svc = _build_service_for_phase4()
    path = _make_path_dto()

    svc.plan_all([[path]])
    svc._planning_thread.join(timeout=2.0)
    list(svc.iterate_planned_trajectories())
    q1 = svc._plan_queue

    svc.plan_all([[path]])
    svc._planning_thread.join(timeout=2.0)
    list(svc.iterate_planned_trajectories())
    q2 = svc._plan_queue

    assert q1 is not q2, 'plan_all() must create a fresh queue.Queue per call (D-04)'


# endregion: Phase 4 — plan_all / iterate / cancel tests
