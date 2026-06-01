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

from unittest.mock import MagicMock, patch

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import CancelGoal
from rclpy.lifecycle.node import TransitionCallbackReturn, LifecycleState
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, MotionSequenceResponse, RobotTrajectory
from moveit_msgs.action import (
    ExecuteTrajectory as MoveItExecuteTrajectory,
    ExecuteTrajectory_GetResult_Response as MoveItExecuteTrajectoryResponse,
)
from movement_controller.msg import TrajectoryPath
from movement_controller.action import ExecuteTrajectory

from movement_controller.models import PlanResultDTO, TrajectoryGoalDTO
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.services import PilzPlannerService
from movement_controller.ur_movement_controller import URMovementController

# Resolve TYPE_CHECKING forward reference so Pydantic can instantiate PlanResultDTO in tests.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': object})

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
    mock_path = MagicMock(spec=TrajectoryPath)
    mock_path.path_id = path_id
    mock_path.motion_type = motion_type
    mock_path.blend_radius = 0.0
    mock_path.target_pose = PoseStamped()
    mock_path.circ_point = Point()
    mock_path.cartesian_speed = 0.0
    mock_path.acceleration = 0.0
    mock_path.tool_frame = ''
    mock_path.circ_type = 'interim'
    goal = MagicMock(spec=ExecuteTrajectory.Goal)
    goal.paths = [mock_path]
    return goal

def _make_path_msg(path_id: str, blend_radius: float = 0.0) -> MagicMock:
    """Build a mock TrajectoryPath message with all fields populated."""
    m = MagicMock(spec=TrajectoryPath)
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


def _make_mock_exec_client(success: bool = True):
    """Build a mock ActionClient for /move_group/execute_trajectory.

    Returns a client whose send_goal() returns a GetResult.Response-shaped mock:
    ``result.result.error_code.val`` is the integer error code, matching the real
    rclpy ActionClient.send_goal() return type.
    """
    mock_result = MoveItExecuteTrajectoryResponse(
        status=4 if success else 3,  # 4=SUCCEEDED, 3=ABORTED
        result=MagicMock(spec=MoveItExecuteTrajectory.Result),
    )
    mock_result.result.error_code.val = (
        MoveItErrorCodes.SUCCESS if success else MoveItErrorCodes.FAILURE
    )
    mock_client = MagicMock(spec=ActionClient)
    mock_client.wait_for_server.return_value = True
    mock_client.send_goal.return_value = mock_result
    return mock_client


def _make_plan_result_dto(path_ids: list, success: bool = True, error_message: str = ''):
    """Build a PlanResultDTO with a mock motion_plan for use in execute_callback tests.

    Successful DTOs include a mock MotionSequenceResponse with one planned trajectory
    so that the execution loop can iterate over planned_trajectories.
    """
    if success:
        mock_plan = MagicMock(spec=MotionSequenceResponse)
        mock_plan.planned_trajectories = [MagicMock(spec=RobotTrajectory)]
    else:
        mock_plan = None
    return PlanResultDTO(
        success=success,
        path_ids=path_ids,
        blended=False,
        motion_plan=mock_plan,
        error_message=error_message,
    )


def _make_mock_planner_with_results(plan_results: list):
    """Build a mock PilzPlannerService that yields the given PlanResultDTOs."""
    mock_planner = MagicMock(spec=PilzPlannerService)
    mock_planner.plan_all.return_value = True
    mock_planner.iterate_planned_trajectories = MagicMock(return_value=iter(plan_results))
    return mock_planner

# region: goal execution
def test_execute_aborts_when_planner_not_configured(node):
    """_execute_callback aborts and returns success=False when _planner_service is None."""
    assert node._planner_service is None
    mock_goal_handle = MagicMock(spec=ServerGoalHandle)
    mock_goal_handle.is_cancel_requested = False
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1)]

    result = node._execute_callback(mock_goal_handle)

    assert result.success is False
    assert result.error_message != ''
    mock_goal_handle.abort.assert_called_once()


def test_goal_rejected_when_executing(node):
    """Goal is rejected when another goal is already executing."""
    node._is_executing = True
    try:
        result = node._goal_callback(_make_ros_goal())
        assert result == GoalResponse.REJECT
    finally:
        node._is_executing = False


def test_goal_rejected_empty_paths(node):
    """Goal is rejected when the paths list is empty."""
    goal = MagicMock(spec=ExecuteTrajectory.Goal)
    goal.paths = []
    result = node._goal_callback(goal)
    assert result == GoalResponse.REJECT


def test_goal_rejected_empty_path_id(node):
    """Goal is rejected when a path has an empty path_id."""
    result = node._goal_callback(_make_ros_goal(path_id=''))
    assert result == GoalResponse.REJECT


def test_goal_rejected_invalid_uuid4_path_id(node):
    """Goal is rejected when a path_id is not a valid UUID4 string."""
    result = node._goal_callback(_make_ros_goal(path_id='not-a-uuid'))
    assert result == GoalResponse.REJECT


def test_goal_rejected_invalid_motion_type(node):
    """Goal is rejected when motion_type is not a valid MotionTypeEnum value."""
    result = node._goal_callback(_make_ros_goal(motion_type='JUMP'))
    assert result == GoalResponse.REJECT


def test_goal_accepted_when_active_valid(node):
    """Goal is accepted when node is active, not executing, and goal is valid."""
    node._is_executing = False
    result = node._goal_callback(_make_ros_goal())
    assert result == GoalResponse.ACCEPT
    node._is_executing = False  # reset after test

def test_execute_callback_stub_feedback_sequence(node):
    """Two paths (br=0.0 each) → 2 groups → 4 feedback messages (executing+completed per group)."""
    dto1 = _make_plan_result_dto([_UUID1])
    dto2 = _make_plan_result_dto([_UUID2])

    node._planner_service = _make_mock_planner_with_results([dto1, dto2])
    node._execute_trajectory_client = _make_mock_exec_client(success=True)

    mock_goal_handle = MagicMock(spec=ServerGoalHandle)
    mock_goal_handle.is_cancel_requested = False
    mock_goal_handle.request.paths = [
        _make_path_msg(_UUID1, 0.0),
        _make_path_msg(_UUID2, 0.0),
    ]

    result = node._execute_callback(mock_goal_handle)

    node._planner_service = None
    node._execute_trajectory_client = None

    assert result.success is True
    assert mock_goal_handle.publish_feedback.call_count == 4
    assert result.trajectory_paths_completed == [_UUID1, _UUID2]
    assert mock_goal_handle.succeed.call_count == 1


def test_execute_callback_clears_is_executing_after_success(node):
    """_is_executing must be False after a successful callback run."""
    dto = _make_plan_result_dto([_UUID1])
    node._planner_service = _make_mock_planner_with_results([dto])
    node._execute_trajectory_client = _make_mock_exec_client(success=True)

    node._is_executing = True  # simulate _goal_callback having set it
    mock_goal_handle = MagicMock(spec=ServerGoalHandle)
    mock_goal_handle.is_cancel_requested = False
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 0.0)]

    node._execute_callback(mock_goal_handle)

    node._planner_service = None
    node._execute_trajectory_client = None

    assert node._is_executing is False


def test_execute_callback_clears_is_executing_after_failure(node):
    """_is_executing must be False even when planning fails."""
    dto = _make_plan_result_dto([_UUID1], success=False, error_message='planning failed')
    node._planner_service = _make_mock_planner_with_results([dto])
    node._execute_trajectory_client = _make_mock_exec_client(success=True)

    node._is_executing = True  # simulate _goal_callback having set it
    mock_goal_handle = MagicMock(spec=ServerGoalHandle)
    mock_goal_handle.is_cancel_requested = False
    mock_goal_handle.request.paths = [_make_path_msg(_UUID1, 0.0)]

    result = node._execute_callback(mock_goal_handle)

    node._planner_service = None
    node._execute_trajectory_client = None

    assert node._is_executing is False
    assert result.success is False
    mock_goal_handle.abort.assert_called_once()

# endregion: goal execution

# region: cancel callback

def test_cancel_callback_returns_accept(node):
    """_cancel_callback() always returns CancelResponse.ACCEPT."""
    result = node._cancel_callback(MagicMock(spec=CancelGoal.Request))
    assert result == CancelResponse.ACCEPT


def test_cancel_callback_calls_planner_cancel_when_configured(node):
    """_cancel_callback() calls planner.cancel() when a planner service is set."""
    mock_planner = MagicMock(spec=PilzPlannerService)
    node._planner_service = mock_planner

    result = node._cancel_callback(MagicMock(spec=CancelGoal.Request))

    mock_planner.cancel.assert_called_once()
    assert result == CancelResponse.ACCEPT
    node._planner_service = None


def test_cancel_callback_safe_when_planner_not_configured(node):
    """_cancel_callback() does not raise and still returns ACCEPT when _planner_service is None."""
    assert node._planner_service is None
    result = node._cancel_callback(MagicMock(spec=CancelGoal.Request))
    assert result == CancelResponse.ACCEPT

# endregion: cancel callback

# region: lifecycle node callbacks
def _set_array_constraint_params(node) -> None:
    """Set STRING_ARRAY / DOUBLE_ARRAY constraint params to empty lists.

    These parameters are declared without defaults in URMovementController.__init__
    (added in Phase 5 Plan 01). Tests that call on_configure() must initialize them
    to avoid ParameterUninitializedException.
    """
    node.set_parameters([
        Parameter('constraints.joint.names', Parameter.Type.STRING_ARRAY, []),
        Parameter('constraints.joint.lower_limits', Parameter.Type.DOUBLE_ARRAY, []),
        Parameter('constraints.joint.upper_limits', Parameter.Type.DOUBLE_ARRAY, []),
        Parameter('constraints.joint.max_velocities', Parameter.Type.DOUBLE_ARRAY, []),
    ])


def test_on_configure_creates_planner_service(node):
    """on_configure() instantiates PilzPlannerService and stores it on the node."""
    assert node._planner_service is None
    _set_array_constraint_params(node)
    mock_state = LifecycleState(label='unconfigured', state_id=0)

    with patch(
        'movement_controller.ur_movement_controller.PilzPlannerService',
        return_value=MagicMock(spec=PilzPlannerService),
    ):
        result = node.on_configure(mock_state)

    assert result == TransitionCallbackReturn.SUCCESS
    assert node._planner_service is not None
    node._planner_service = None  # reset


def test_on_configure_uses_moveit_group_name_parameter(node):
    """on_configure() passes the moveit_group_name parameter value to PilzPlannerService."""
    _set_array_constraint_params(node)
    mock_state = LifecycleState(label='unconfigured', state_id=0)

    with patch(
        'movement_controller.ur_movement_controller.PilzPlannerService',
    ) as patched_cls:
        patched_cls.return_value = MagicMock(spec=PilzPlannerService)
        node.on_configure(mock_state)
        _, kwargs = patched_cls.call_args
        assert kwargs.get('moveit_group_name') == 'ur_manipulator'

    node._planner_service = None  # reset


def test_on_activate_returns_failure_when_planner_not_configured(node):
    """on_activate() returns FAILURE when _planner_service is None (not yet configured)."""
    assert node._planner_service is None
    mock_state = LifecycleState(label='inactive', state_id=1)

    result = node.on_activate(mock_state)

    assert result == TransitionCallbackReturn.FAILURE


def test_on_activate_returns_failure_when_service_unavailable(node):
    """on_activate() returns FAILURE when wait_for_service times out."""
    mock_planner = MagicMock(spec=PilzPlannerService)
    mock_planner.wait_for_service.return_value = False
    node._planner_service = mock_planner
    mock_state = LifecycleState(label='inactive', state_id=1)

    result = node.on_activate(mock_state)

    assert result == TransitionCallbackReturn.FAILURE
    mock_planner.on_activate.assert_called_once()
    mock_planner.wait_for_service.assert_called_once()
    node._planner_service = None  # reset


def test_on_activate_calls_planner_on_activate_and_wait_for_service(node):
    """on_activate() calls planner.on_activate() then planner.wait_for_service()."""
    mock_planner = MagicMock(spec=PilzPlannerService)
    mock_planner.wait_for_service.return_value = True
    node._planner_service = mock_planner
    mock_state = LifecycleState(label='inactive', state_id=1)

    with patch('movement_controller.ur_movement_controller.ActionServer'), \
         patch('movement_controller.ur_movement_controller.ActionClient'):
        result = node.on_activate(mock_state)

    mock_planner.on_activate.assert_called_once()
    mock_planner.wait_for_service.assert_called_once()
    assert result == TransitionCallbackReturn.SUCCESS
    # Cleanup references set inside on_activate
    node._action_server = None
    node._execute_trajectory_client = None
    node._planner_service = None


def test_on_deactivate_cancels_and_deactivates_planner(node):
    """on_deactivate() calls planner.cancel() then planner.on_deactivate()."""
    mock_planner = MagicMock(spec=PilzPlannerService)
    node._planner_service = mock_planner
    node._is_executing = True
    mock_state = LifecycleState(label='active', state_id=2)

    result = node.on_deactivate(mock_state)


    assert result == TransitionCallbackReturn.SUCCESS
    assert node._is_executing is False
    assert node._trajectory_goal is None
    node._planner_service = None


def test_on_deactivate_destroys_action_server(node):
    """on_deactivate() destroys the action server and clears the reference."""
    mock_action_server = MagicMock(spec=ActionServer)
    node._action_server = mock_action_server
    mock_state = LifecycleState(label='active', state_id=2)

    node.on_deactivate(mock_state)

    mock_action_server.destroy.assert_called_once()
    assert node._action_server is None


def test_on_deactivate_destroys_execute_trajectory_client(node):
    """on_deactivate() destroys the execute trajectory client and clears the reference."""
    mock_client = MagicMock(spec=ActionClient)
    node._execute_trajectory_client = mock_client
    mock_state = LifecycleState(label='active', state_id=2)

    node.on_deactivate(mock_state)

    mock_client.destroy.assert_called_once()
    assert node._execute_trajectory_client is None


def test_on_cleanup_clears_planner_service(node):
    """on_cleanup() sets _planner_service to None."""
    node._planner_service = MagicMock(spec=PilzPlannerService)
    mock_state = LifecycleState(label='inactive', state_id=1)

    result = node.on_cleanup(mock_state)

    assert result == TransitionCallbackReturn.SUCCESS
    assert node._planner_service is None


def test_on_cleanup_clears_trajectory_goal(node):
    """on_cleanup() sets _trajectory_goal to None."""
    node._trajectory_goal = MagicMock(spec=TrajectoryGoalDTO)
    mock_state = LifecycleState(label='inactive', state_id=1)

    node.on_cleanup(mock_state)

    assert node._trajectory_goal is None


def test_on_deactivate_is_idempotent_when_resources_are_none(node):
    """on_deactivate() succeeds gracefully when all resources are already None."""
    assert node._planner_service is None
    assert node._action_server is None
    assert node._execute_trajectory_client is None
    mock_state = LifecycleState(label='active', state_id=2)

    result = node.on_deactivate(mock_state)

    assert result == TransitionCallbackReturn.SUCCESS

# endregion: lifecycle node callbacks


# region: speed and acceleration cap enforcement tests

def test_goal_rejected_when_cartesian_speed_exceeds_max(node):
    """_goal_callback rejects when path.cartesian_speed > max_cartesian_speed."""
    node._constraint_config = ConstraintConfigDTO(max_cartesian_speed=0.5)
    goal = _make_ros_goal()
    goal.paths[0].cartesian_speed = 0.8  # exceeds 0.5 cap
    try:
        result = node._goal_callback(goal)
        assert result == GoalResponse.REJECT
        assert node._is_executing is False
    finally:
        node._constraint_config = None


def test_goal_accepted_when_cartesian_speed_within_max(node):
    """_goal_callback accepts when path.cartesian_speed <= max_cartesian_speed."""
    node._constraint_config = ConstraintConfigDTO(max_cartesian_speed=0.5)
    goal = _make_ros_goal()
    goal.paths[0].cartesian_speed = 0.3  # within cap
    try:
        result = node._goal_callback(goal)
        assert result == GoalResponse.ACCEPT
    finally:
        node._is_executing = False
        node._constraint_config = None


def test_goal_accepted_when_max_cartesian_speed_is_zero(node):
    """max_cartesian_speed=0.0 (sentinel) → speed check skipped, any speed accepted."""
    node._constraint_config = ConstraintConfigDTO(max_cartesian_speed=0.0)
    goal = _make_ros_goal()
    goal.paths[0].cartesian_speed = 999.0
    try:
        result = node._goal_callback(goal)
        assert result == GoalResponse.ACCEPT
    finally:
        node._is_executing = False
        node._constraint_config = None


def test_goal_rejected_when_acceleration_exceeds_max(node):
    """_goal_callback rejects when path.acceleration > max_acceleration."""
    node._constraint_config = ConstraintConfigDTO(max_acceleration=0.3)
    goal = _make_ros_goal()
    goal.paths[0].acceleration = 0.5  # exceeds 0.3 cap
    try:
        result = node._goal_callback(goal)
        assert result == GoalResponse.REJECT
        assert node._is_executing is False
    finally:
        node._constraint_config = None


def test_goal_accepted_when_constraint_config_is_none(node):
    """When _constraint_config is None, no speed check is performed."""
    assert node._constraint_config is None
    goal = _make_ros_goal()
    goal.paths[0].cartesian_speed = 999.0
    goal.paths[0].acceleration = 999.0
    result = node._goal_callback(goal)
    assert result == GoalResponse.ACCEPT
    node._is_executing = False


def test_goal_accepted_when_path_speed_is_zero(node):
    """path.cartesian_speed=0.0 → check skipped (0.0 means unspecified in path)."""
    node._constraint_config = ConstraintConfigDTO(max_cartesian_speed=0.5)
    goal = _make_ros_goal()
    goal.paths[0].cartesian_speed = 0.0  # unspecified
    try:
        result = node._goal_callback(goal)
        assert result == GoalResponse.ACCEPT
    finally:
        node._is_executing = False
        node._constraint_config = None

# endregion: speed and acceleration cap enforcement tests