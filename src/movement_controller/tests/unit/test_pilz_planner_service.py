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

from unittest.mock import MagicMock, call, patch

import pytest
from geometry_msgs.msg import Point, PoseStamped

from movement_controller.enums.circ_type_enum import CircTypeEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService

_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'


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
def service(mock_moveit, mock_planning_component):
    """PilzPlannerService with injected mocks."""
    return PilzPlannerService(mock_moveit, mock_planning_component)


@pytest.fixture(autouse=True)
def patch_plan_request_params():
    """Patch PlanRequestParameters so no real MoveItPy constructor is invoked."""
    with patch(
        'movement_controller.services.pilz_planner_service.PlanRequestParameters'
    ) as mock_cls:
        yield mock_cls


# ---------------------------------------------------------------------------
# Success-path tests
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# Failure-path tests
# ---------------------------------------------------------------------------


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
