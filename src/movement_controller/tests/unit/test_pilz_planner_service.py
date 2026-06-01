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
"""Unit tests for PilzPlannerService — all ROS 2 service dependencies mocked."""

from queue import Queue
from threading import Thread, Event
from time import sleep
from typing import Tuple
from unittest.mock import MagicMock

import pytest
from rclpy.lifecycle import LifecycleNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.client import Client
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState

from moveit_msgs.msg import (
    MoveItErrorCodes,
    RobotState,
    RobotTrajectory
)
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.srv import GetPlanningScene
from trajectory_msgs.msg import JointTrajectoryPoint

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.exceptions.abort_planning_error import AbortPlanningError
from movement_controller.exceptions.not_initialized_error import NotInitializedError
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.planning_session_dto import PlanningSessionDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService


# Resolve TYPE_CHECKING-only forward references so Pydantic can instantiate these models in tests.
# Use 'MagicMock' as the resolved type so it is accepted by the validator.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': MagicMock})
PlanningSessionDTO.model_rebuild(_types_namespace={'RobotState': MagicMock})

_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID2 = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'

_SEQ_SRV = '/move_group/plan_sequence_path'
_SCENE_SRV = '/move_group/get_planning_scene'


def _make_path_dto(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with sensible defaults."""
    defaults: dict = {
        'path_id': _UUID,
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


def _make_sync_future(response):
    """Return a mock ROS future that invokes add_done_callback synchronously."""
    future = MagicMock()
    future.result.return_value = response

    def add_done_callback(cb):
        cb(future)

    future.add_done_callback = add_done_callback
    return future


def _make_default_seq_response():
    """Build a default successful GetMotionSequence response mock."""
    resp = MagicMock(spec=MoveGroupSequence.Result)
    resp.response.error_code.val = MoveItErrorCodes.SUCCESS
    mock_traj = MagicMock(spec=RobotTrajectory)
    mock_traj.joint_trajectory.joint_names = ['joint_1']
    joint_traj_point = MagicMock(spec=JointTrajectoryPoint)
    joint_traj_point.positions = [0.0]
    mock_traj.joint_trajectory.points = [joint_traj_point]
    resp.response.planned_trajectories = [mock_traj]
    return resp


def _make_default_scene_response():
    """Build a default successful GetPlanningScene response mock."""
    resp = MagicMock(spec=GetPlanningScene.Response)
    resp.scene.robot_state = MagicMock(spec=RobotState)
    return resp


def _build_service(
    seq_response=None,
    scene_response=None,
    scene_available=True,
    seq_available=True
) -> Tuple[PilzPlannerService, MagicMock, MagicMock]:
    """Build an activated PilzPlannerService backed by synchronous mock service clients.

    Sync futures cause the callback chain to execute immediately inside plan_all(),
    so the queue is fully populated before plan_all() returns.
    """
    seq_resp = seq_response if seq_response is not None else _make_default_seq_response()
    scene_resp = scene_response if scene_response is not None else _make_default_scene_response()

    mock_seq_client = MagicMock()
    mock_seq_client.wait_for_service.return_value = seq_available
    mock_seq_client.call_async.side_effect = lambda req: _make_sync_future(seq_resp)

    mock_scene_client = MagicMock()
    mock_scene_client.wait_for_service.return_value = scene_available
    mock_scene_client.call_async.side_effect = lambda req: _make_sync_future(scene_resp)

    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)

    def create_client_side_effect(**kwargs):
        if kwargs.get('srv_name') == _SCENE_SRV:
            return mock_scene_client
        return mock_seq_client

    mock_node.create_client.side_effect = create_client_side_effect

    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()
    return svc, mock_seq_client, mock_scene_client


# region: Constructor and lifecycle tests
def test_constructor_stores_group_name():
    """Constructor stores the planning group name for use in service requests."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc._group_name == 'ur_manipulator'
    mock_node.get_logger.assert_called_once()


def test_constructor_no_clients_before_activate():
    """Service clients are NOT created in __init__; they are created in on_activate()."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc._plan_seq_client is None
    assert svc._scene_monitor_client is None
    mock_node.create_client.assert_not_called()
    mock_node.get_logger.assert_called_once()


def test_on_activate_creates_clients():
    """on_activate() creates both service clients via node.create_client."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = MagicMock(spec=Client)

    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()

    assert mock_node.create_client.call_count == 2
    service_names = [call.kwargs['srv_name'] for call in mock_node.create_client.call_args_list]
    assert _SEQ_SRV in service_names
    assert _SCENE_SRV in service_names
    assert svc._plan_seq_client is not None
    assert svc._scene_monitor_client is not None


def test_on_deactivate_destroys_clients():
    """on_deactivate() destroys both service clients and clears the references."""
    svc, _, _ = _build_service()
    seq_client = svc._plan_seq_client
    scene_client = svc._scene_monitor_client

    svc.on_deactivate()

    svc._node.destroy_client.assert_any_call(seq_client)  # type: ignore
    svc._node.destroy_client.assert_any_call(scene_client)  # type: ignore
    assert svc._plan_seq_client is None
    assert svc._scene_monitor_client is None

# endregion: Constructor and lifecycle tests

# region: wait_for_service tests
def test_wait_for_service_returns_false_before_activate():
    """wait_for_service() returns False when called before on_activate() (client is None)."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc.wait_for_service(timeout_sec=1.0) is False


def test_wait_for_service_delegates_to_client():
    """wait_for_service() delegates to the plan_seq_client with the given timeout."""
    svc, mock_seq_client, _ = _build_service()
    mock_seq_client.wait_for_service.return_value = True

    result = svc.wait_for_service(timeout_sec=3.0)

    assert result is True
    mock_seq_client.wait_for_service.assert_called_with(timeout_sec=3.0)

# endregion: wait_for_service tests

# region: plan_all tests
def test_plan_all_returns_false_when_scene_service_unavailable():
    """plan_all() returns False and does not start planning if scene service is unavailable."""
    svc, _, _ = _build_service(scene_available=False)

    result = svc.plan_all([[_make_path_dto()]])

    assert result is False
    assert svc._plan_queue is None


def test_plan_all_returns_true_and_enqueues_results():
    """plan_all() returns True and the callback chain populates the queue synchronously."""
    svc, _, _ = _build_service()

    result = svc.plan_all([[_make_path_dto()]])

    assert result is True
    assert svc._plan_queue is not None
    results = list(svc.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].success is True


def test_plan_all_creates_fresh_queue_per_call():
    """plan_all() creates a new queue.Queue on each call (fresh per goal invocation)."""
    svc, _, _ = _build_service()
    path = _make_path_dto()

    svc.plan_all([[path]])
    q1 = svc._plan_queue
    list(svc.iterate_planned_trajectories())

    svc.plan_all([[path]])
    q2 = svc._plan_queue
    list(svc.iterate_planned_trajectories())

    assert q1 is not q2, 'plan_all() must create a fresh queue.Queue per call'

# endregion: plan_all tests

# region: iterate_planned_trajectories tests
def test_iterate_raises_not_initialized_before_plan_all():
    """iterate_planned_trajectories() raises NotInitializedError when called before plan_all()."""
    svc, _, _ = _build_service()
    with pytest.raises(NotInitializedError):
        next(svc.iterate_planned_trajectories())


def test_iterate_yields_single_path_result():
    """iterate_planned_trajectories yields PlanResultDTOs from the queue, then terminates."""
    svc, _, _ = _build_service()
    svc._plan_queue = Queue()
    svc._cancel_event = Event()

    dto = PlanResultDTO(success=True, path_ids=[_UUID], blended=False)
    svc._plan_queue.put(dto)
    svc._plan_queue.put(StopIteration())

    results = list(svc.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].path_ids == [_UUID]
    assert results[0].blended is False


def test_iterate_blended_group_sets_blended_true():
    """Two-path group result with blended=True and both path IDs is yielded correctly."""
    svc, _, _ = _build_service()
    svc._plan_queue = Queue()
    svc._cancel_event = Event()

    dto = PlanResultDTO(
        success=True,
        path_ids=[_UUID, _UUID2],
        blended=True,
    )
    svc._plan_queue.put(dto)
    svc._plan_queue.put(StopIteration())

    results = list(svc.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].blended is True
    assert results[0].path_ids == [_UUID, _UUID2]

# endregion: iterate_planned_trajectories tests

# region: cancel tests
def test_cancel_terminates_iterator_cleanly():
    """cancel() while the iterator is blocked on an empty queue causes it to unblock via AbortPlanningError."""
    # Use a non-sync future so the callback never fires — queue remains empty after plan_all()
    blocking_future = MagicMock()

    def never_call_callback(cb):
        pass  # intentionally do not call the callback

    blocking_future.add_done_callback = never_call_callback

    mock_seq_client = MagicMock(spec=Client)
    mock_seq_client.wait_for_service.return_value = True
    mock_seq_client.call_async.return_value = blocking_future

    mock_scene_client = MagicMock(spec=Client)
    mock_scene_client.wait_for_service.return_value = True
    mock_scene_client.call_async.return_value = blocking_future

    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)

    def create_client_side_effect(**kwargs):
        if kwargs.get('srv_name') == _SCENE_SRV:
            return mock_scene_client
        return mock_seq_client

    mock_node.create_client.side_effect = create_client_side_effect

    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()
    svc.plan_all([[_make_path_dto()]])

    # Cancel after a brief delay to unblock the iterator
    def do_cancel():
        sleep(0.05)
        svc.cancel()

    Thread(target=do_cancel, daemon=True).start()

    # Iterator should terminate (via AbortPlanningError) within 2 seconds
    done_event = Event()

    def drain():
        try:
            list(svc.iterate_planned_trajectories())
        except AbortPlanningError:
            pass
        done_event.set()

    Thread(target=drain, daemon=True).start()
    assert done_event.wait(timeout=2.0), 'Iterator did not terminate after cancel() — possible hang'


# endregion: cancel tests

# region: _generate_motion_sequence_request tests

def test_last_item_blend_radius_forced_to_zero():
    """In a 2-path group the last MotionSequenceItem always has blend_radius=0.0 (PILZ constraint)."""
    svc, _, _ = _build_service()
    path1 = _make_path_dto(path_id=_UUID, blend_radius=0.05)
    path2 = _make_path_dto(path_id=_UUID2, blend_radius=0.05)

    seq_req = svc._generate_motion_sequence_request([path1, path2], RobotState())  # type: ignore

    assert len(seq_req.items) == 2
    assert seq_req.items[0].blend_radius == 0.05, 'First item should keep its original blend_radius' # type: ignore
    assert seq_req.items[-1].blend_radius == 0.0, 'Last item blend_radius must be forced to 0.0'  # type: ignore


def test_generate_request_maps_motion_type_to_planner_id():
    """_generate_motion_sequence_request maps MotionTypeEnum to the correct PILZ planner_id."""
    svc, _, _ = _build_service()
    for motion_type, expected_planner_id in [
        (MotionTypeEnum.LIN, 'LIN'),
        (MotionTypeEnum.PTP, 'PTP'),
    ]:
        path = _make_path_dto(motion_type=motion_type)
        seq_req = svc._generate_motion_sequence_request([path], RobotState())
        assert seq_req.items[0].req.planner_id == expected_planner_id  # type: ignore
        assert seq_req.items[0].req.pipeline_id == 'pilz_industrial_motion_planner'  # type: ignore


def test_generate_request_sets_start_state_on_first_item_only():
    """start_state is assigned only to the first MotionSequenceItem."""
    svc, _, _ = _build_service()
    start = RobotState()
    path1 = _make_path_dto(path_id=_UUID)
    path2 = _make_path_dto(path_id=_UUID2)

    seq_req = svc._generate_motion_sequence_request([path1, path2], start)  # type: ignore

    assert seq_req.items[0].req.start_state is start  # type: ignore
    # Subsequent items should not be the same object as start
    assert seq_req.items[1].req.start_state is not start  # type: ignore

# endregion: _generate_motion_sequence_request tests

# region: planning-failure / error-path tests
def test_planning_failure_raises_abort_error():
    """A GetMotionSequence response with error_code != SUCCESS causes AbortPlanningError."""
    fail_resp = MagicMock(spec=MoveGroupSequence.Result)
    fail_resp.response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
    svc, _, _ = _build_service(seq_response=fail_resp)

    svc.plan_all([[_make_path_dto()]])

    with pytest.raises(AbortPlanningError):
        list(svc.iterate_planned_trajectories())


def test_scene_retrieval_failure_raises_abort_error():
    """A GetPlanningScene response with no robot_state causes AbortPlanningError."""
    scene_resp = MagicMock(spec=GetPlanningScene.Response)
    scene_resp.scene.robot_state = None  # triggers the guard in _initiate_planning
    svc, _, _ = _build_service(scene_response=scene_resp)

    svc.plan_all([[_make_path_dto()]])

    with pytest.raises(AbortPlanningError):
        list(svc.iterate_planned_trajectories())

# endregion: planning-failure / error-path tests
