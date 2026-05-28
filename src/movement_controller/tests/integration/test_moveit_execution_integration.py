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
"""Integration smoke test — URMovementController with MoveItPy mocked at module level.

Per D-19: no real move_group node is required. All MoveItPy interactions are
performed via mocks injected at the Python module level. This test can run in
`colcon test` without a robot or ROS2 move_group stack.
"""

import asyncio
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import GoalResponse

from movement_controller.ur_movement_controller import URMovementController

_UUID1 = '00000000-0000-4000-8000-000000000001'


def _make_path_msg(path_id=_UUID1, motion_type='LIN', circ_type='interim'):
    """Build a mock TrajectoryPath message with all fields populated."""
    msg = MagicMock()
    msg.path_id = path_id
    msg.motion_type = motion_type
    msg.blend_radius = 0.0
    msg.target_pose = PoseStamped()
    msg.circ_point = Point()
    msg.cartesian_speed = 0.0
    msg.acceleration = 0.0
    msg.tool_frame = ''
    msg.circ_type = circ_type
    return msg


def _make_goal_with_circ_path(circ_type: str):
    """Build a mock goal with a single CIRC path and the given circ_type."""
    goal = MagicMock()
    path = MagicMock()
    path.path_id = _UUID1
    path.motion_type = 'CIRC'
    path.circ_type = circ_type
    path.blend_radius = 0.0
    path.target_pose = PoseStamped()
    path.circ_point = Point()
    path.cartesian_speed = 0.0
    path.acceleration = 0.0
    path.tool_frame = ''
    goal.paths = [path]
    return goal


@pytest.fixture(scope='module')
def ros_context():
    """Initialise rclpy once per test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node_with_moveit(ros_context):
    """URMovementController with MoveItPy mocked at module level and services injected."""
    mock_plan_result = MagicMock()
    mock_plan_result.__bool__ = MagicMock(return_value=True)
    mock_plan_result.trajectory = MagicMock()

    mock_exec_status = MagicMock()
    mock_exec_status.__bool__ = MagicMock(return_value=True)

    mock_moveit = MagicMock()
    mock_moveit.execute.return_value = mock_exec_status

    with patch('movement_controller.ur_movement_controller.MoveItPy', return_value=mock_moveit):
        n = URMovementController()

    n._is_active = True
    n._moveit = mock_moveit

    # Inject mock planner service that returns success by default
    mock_planner = MagicMock()
    mock_planner.plan.return_value = MagicMock(
        success=True,
        trajectory=MagicMock(),
        error_message='',
    )
    n._planner_service = mock_planner

    yield n
    n.destroy_node()


# ---------------------------------------------------------------------------
# _execute_callback tests
# ---------------------------------------------------------------------------

def test_execute_trajectory_single_path_success(node_with_moveit):
    """1-path LIN goal → result.success=True, 2 feedback calls, path_id in completed list."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'LIN')]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is True
    assert result.trajectory_paths_completed == [_UUID1]
    assert mock_goal_handle.publish_feedback.call_count == 2
    mock_goal_handle.succeed.assert_called_once()


def test_execute_trajectory_feedback_order(node_with_moveit):
    """Per-path feedback order: executing first, then completed (D-15)."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'LIN')]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    calls = mock_goal_handle.publish_feedback.call_args_list
    assert len(calls) == 2
    first_fb = calls[0].args[0]
    second_fb = calls[1].args[0]
    assert first_fb.status == 'executing'
    assert second_fb.status == 'completed'


def test_execute_trajectory_aborts_on_plan_failure(node_with_moveit, monkeypatch):
    """Planning failure → result.success=False, path_id in error_message, abort called."""
    error_msg = f'PILZ LIN planning failed for path {_UUID1!r}'
    monkeypatch.setattr(
        node_with_moveit._planner_service,
        'plan',
        MagicMock(return_value=MagicMock(success=False, trajectory=None, error_message=error_msg)),
    )

    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'LIN')]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.abort = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is False
    assert _UUID1 in result.error_message
    mock_goal_handle.abort.assert_called_once()
    mock_goal_handle.succeed.assert_not_called()


def test_execute_trajectory_aborts_on_execution_failure(node_with_moveit, monkeypatch):
    """Execution failure → result.success=False, path_id in error_message, abort called."""
    failing_exec = MagicMock()
    failing_exec.__bool__ = MagicMock(return_value=False)
    monkeypatch.setattr(node_with_moveit._moveit, 'execute', MagicMock(return_value=failing_exec))

    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'LIN')]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.abort = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is False
    assert _UUID1 in result.error_message
    mock_goal_handle.abort.assert_called_once()


def test_execute_trajectory_circ_path_success(node_with_moveit):
    """CIRC path with valid circ_type='interim' → result.success=True (D-15)."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 'CIRC', circ_type='interim')]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node_with_moveit._execute_callback(mock_goal_handle))

    assert result.success is True


# ---------------------------------------------------------------------------
# _goal_callback tests
# ---------------------------------------------------------------------------

def test_goal_rejects_circ_with_empty_circ_type(node_with_moveit):
    """CIRC path with circ_type='' → GoalResponse.REJECT (D-11); _is_executing stays False."""
    node_with_moveit._is_executing = False
    goal = _make_goal_with_circ_path(circ_type='')

    result = node_with_moveit._goal_callback(goal)

    assert result == GoalResponse.REJECT
    assert node_with_moveit._is_executing is False


def test_goal_rejects_concurrent_execution(node_with_moveit):
    """Concurrent goal rejected when _is_executing=True (MOT-05)."""
    node_with_moveit._is_executing = True
    valid_goal = MagicMock()
    valid_goal.paths = [_make_path_msg(_UUID1, 'LIN')]

    result = node_with_moveit._goal_callback(valid_goal)

    node_with_moveit._is_executing = False  # clean up

    assert result == GoalResponse.REJECT


def test_goal_rejects_circ_with_unrecognized_circ_type(node_with_moveit):
    """CIRC path with circ_type='unknown' → GoalResponse.REJECT."""
    node_with_moveit._is_executing = False
    goal = _make_goal_with_circ_path(circ_type='unknown')

    result = node_with_moveit._goal_callback(goal)

    assert result == GoalResponse.REJECT
    assert node_with_moveit._is_executing is False
