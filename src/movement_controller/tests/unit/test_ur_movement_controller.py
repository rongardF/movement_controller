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
from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import GoalResponse

from movement_controller.ur_movement_controller import URMovementController

# Predefined valid UUID4 values for deterministic tests.
_UUID1 = '00000000-0000-4000-8000-000000000001'
_UUID2 = '00000000-0000-4000-8000-000000000002'


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


def _make_ros_goal(path_id=_UUID1, motion_type='LIN'):
    """Build a mock ExecuteTrajectory.Goal with a single fully-populated path."""
    mock_path = MagicMock()
    mock_path.path_id = path_id
    mock_path.motion_type = motion_type
    mock_path.blend_radius = 0.0
    mock_path.target_pose = PoseStamped()
    mock_path.circ_point = Point()
    mock_path.cartesian_speed = 0.0
    mock_path.acceleration = 0.0
    mock_path.tool_frame = ''
    mock_path.circ_type = 'interim'
    goal = MagicMock()
    goal.paths = [mock_path]
    return goal


def test_goal_rejected_when_not_active(node):
    """Goal is rejected when _is_active is False (default for a new node)."""
    assert node._is_active is False
    result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.REJECT


def test_goal_rejected_when_executing(node):
    """Goal is rejected when another goal is already executing."""
    node._is_active = True
    node._is_executing = True
    try:
        result = node._goal_callback(_make_ros_goal())
        assert result == GoalResponse.REJECT
    finally:
        node._is_executing = False


def test_goal_rejected_empty_paths(node):
    """Goal is rejected when the paths list is empty."""
    node._is_active = True
    goal = MagicMock()
    goal.paths = []
    result = node._goal_callback(goal)
    assert result == GoalResponse.REJECT


def test_goal_rejected_empty_path_id(node):
    """Goal is rejected when a path has an empty path_id."""
    node._is_active = True
    result = node._goal_callback(_make_ros_goal(path_id=''))
    assert result == GoalResponse.REJECT


def test_goal_rejected_invalid_uuid4_path_id(node):
    """Goal is rejected when a path_id is not a valid UUID4 string."""
    node._is_active = True
    result = node._goal_callback(_make_ros_goal(path_id='not-a-uuid'))
    assert result == GoalResponse.REJECT


def test_goal_rejected_invalid_motion_type(node):
    """Goal is rejected when motion_type is not a valid MotionTypeEnum value."""
    node._is_active = True
    result = node._goal_callback(_make_ros_goal(motion_type='JUMP'))
    assert result == GoalResponse.REJECT


def test_goal_accepted_when_active_valid(node):
    """Goal is accepted when node is active, not executing, and goal is valid."""
    node._is_active = True
    node._is_executing = False
    result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.ACCEPT
    node._is_executing = False  # reset after test


def _make_path_msg(path_id: str, blend_radius: float = 0.0) -> MagicMock:
    """Build a mock TrajectoryPath message with all fields populated."""
    m = MagicMock()
    m.path_id = path_id
    m.motion_type = 'LIN'
    m.blend_radius = blend_radius
    m.target_pose = PoseStamped()
    m.circ_point = Point()
    m.cartesian_speed = 0.0
    m.acceleration = 0.0
    m.tool_frame = ''
    m.circ_type = 'interim'
    return m


def test_execute_callback_stub_feedback_sequence(node):
    """Two paths (br=0.0 each) → 2 groups → 4 feedback messages (executing+completed per path)."""
    mock_plan_result = MagicMock()
    mock_plan_result.success = True
    mock_plan_result.trajectory = MagicMock()
    node._planner_service = MagicMock()
    node._planner_service.plan.return_value = mock_plan_result
    mock_exec_status = MagicMock()
    mock_exec_status.__bool__ = MagicMock(return_value=True)
    node._moveit = MagicMock()
    node._moveit.execute.return_value = mock_exec_status

    node._is_active = True
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [
        _make_path_msg(_UUID1, 0.0),
        _make_path_msg(_UUID2, 0.0),
    ]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    result = asyncio.run(node._execute_callback(mock_goal_handle))

    node._planner_service = None
    node._moveit = None

    assert result.success is True
    assert mock_goal_handle.publish_feedback.call_count == 4
    assert result.trajectory_paths_completed == [_UUID1, _UUID2]
    assert mock_goal_handle.succeed.call_count == 1


def test_execute_callback_clears_is_executing_after_success(node):
    """_is_executing must be False after a successful callback run."""
    mock_plan_result = MagicMock()
    mock_plan_result.success = True
    mock_plan_result.trajectory = MagicMock()
    node._planner_service = MagicMock()
    node._planner_service.plan.return_value = mock_plan_result
    mock_exec_status = MagicMock()
    mock_exec_status.__bool__ = MagicMock(return_value=True)
    node._moveit = MagicMock()
    node._moveit.execute.return_value = mock_exec_status

    node._is_executing = True  # simulate _goal_callback having set it
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 0.0)]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()

    asyncio.run(node._execute_callback(mock_goal_handle))

    node._planner_service = None
    node._moveit = None

    assert node._is_executing is False


def test_execute_callback_clears_is_executing_after_failure(node):
    """_is_executing must be False even when planning fails."""
    failing_plan = MagicMock()
    failing_plan.success = False
    failing_plan.error_message = 'planning failed'
    node._planner_service = MagicMock()
    node._planner_service.plan.return_value = failing_plan
    node._moveit = MagicMock()

    node._is_executing = True  # simulate _goal_callback having set it
    mock_goal_handle = MagicMock()
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 0.0)]
    mock_goal_handle.publish_feedback = MagicMock()
    mock_goal_handle.succeed = MagicMock()
    mock_goal_handle.abort = MagicMock()

    result = asyncio.run(node._execute_callback(mock_goal_handle))

    node._planner_service = None
    node._moveit = None

    assert node._is_executing is False
    assert result.success is False
    mock_goal_handle.abort.assert_called_once()
