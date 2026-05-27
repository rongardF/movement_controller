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
"""Unit tests for URMovementController lifecycle and action server callbacks."""

import asyncio
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from lifecycle_msgs.msg import State
from rclpy.action import GoalResponse

from movement_controller.ur_movement_controller import URMovementController


@pytest.fixture(scope='module')
def ros_context():
    """Initialise rclpy once per test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    """Create a fresh URMovementController node for each test."""
    n = URMovementController()
    yield n
    n.destroy_node()


def _make_ros_goal(path_id='uuid-1', motion_type='LIN'):
    """Build a mock ExecuteTrajectory.Goal with a single path."""
    mock_path = MagicMock()
    mock_path.path_id = path_id
    mock_path.motion_type = motion_type
    goal = MagicMock()
    goal.paths = [mock_path]
    return goal


def test_goal_rejected_when_not_active(node):
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_INACTIVE, 'inactive')
        result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.REJECT


def test_goal_rejected_when_unconfigured(node):
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured')
        result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.REJECT


def test_goal_rejected_when_executing(node):
    node._is_executing = True
    try:
        with patch.object(node, '_state_machine') as mock_sm:
            mock_sm.current_state = (State.PRIMARY_STATE_ACTIVE, 'active')
            result = node._goal_callback(_make_ros_goal())
        assert result == GoalResponse.REJECT
    finally:
        node._is_executing = False


def test_goal_rejected_empty_paths(node):
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_ACTIVE, 'active')
        goal = MagicMock()
        goal.paths = []
        result = node._goal_callback(goal)
    assert result == GoalResponse.REJECT


def test_goal_rejected_empty_path_id(node):
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_ACTIVE, 'active')
        result = node._goal_callback(_make_ros_goal(path_id=''))
    assert result == GoalResponse.REJECT


def test_goal_rejected_invalid_motion_type(node):
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_ACTIVE, 'active')
        result = node._goal_callback(_make_ros_goal(motion_type='JUMP'))
    assert result == GoalResponse.REJECT


def test_goal_accepted_when_active_valid(node):
    node._is_executing = False
    with patch.object(node, '_state_machine') as mock_sm:
        mock_sm.current_state = (State.PRIMARY_STATE_ACTIVE, 'active')
        result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.ACCEPT


def _make_path_msg(path_id: str, blend_radius: float = 0.0) -> MagicMock:
    """Build a mock TrajectoryPath message."""
    m = MagicMock()
    m.path_id = path_id
    m.motion_type = 'LIN'
    m.blend_radius = blend_radius
    m.target_pose = PoseStamped()
    m.circ_point = Point()
    m.cartesian_speed = 0.0
    m.acceleration = 0.0
    m.tool_frame = ''
    m.circ_type = ''
    return m


def test_execute_callback_stub_feedback_sequence(node):
    """Two paths (br=0.0 each) → 2 groups → 4 feedback messages."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [
        _make_path_msg('p1', 0.0),
        _make_path_msg('p2', 0.0),
    ]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node._execute_callback(mock_goal_handle))

    assert result.success is True
    assert mock_goal_handle.publish_feedback.call_count == 4
    assert result.trajectory_paths_completed == ['p1', 'p2']
    assert mock_goal_handle.succeed.call_count == 1


def test_execute_callback_clears_is_executing_after_success(node):
    """_is_executing must be False after a successful callback run."""
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg('p1', 0.0)]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    asyncio.run(node._execute_callback(mock_goal_handle))

    assert node._is_executing is False
