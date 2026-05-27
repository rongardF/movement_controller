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
"""Unit tests for MotionTypeEnum, FeedbackStatusEnum, TrajectoryPathDTO, and TrajectoryGoalDTO."""

import pytest
from geometry_msgs.msg import Point, PoseStamped
from pydantic import ValidationError

from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


def _make_path(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with sensible defaults."""
    defaults = {
        'path_id': 'p1',
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
        'circ_point': Point(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


def test_motion_type_enum_values():
    assert MotionTypeEnum.LIN == 'LIN'
    assert MotionTypeEnum.PTP == 'PTP'
    assert MotionTypeEnum.CIRC == 'CIRC'
    assert isinstance(MotionTypeEnum.LIN, str)


def test_feedback_status_enum_values():
    assert FeedbackStatusEnum.EXECUTING == 'executing'
    assert FeedbackStatusEnum.COMPLETED == 'completed'
    assert isinstance(FeedbackStatusEnum.EXECUTING, str)


def test_trajectory_path_dto_valid():
    dto = _make_path(path_id='test-uuid-1234', blend_radius=0.0)
    assert dto.path_id == 'test-uuid-1234'
    assert dto.motion_type == MotionTypeEnum.LIN
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_invalid_motion_type():
    with pytest.raises(ValidationError):
        _make_path(motion_type='INVALID')


def test_trajectory_path_dto_empty_path_id():
    with pytest.raises(ValidationError):
        _make_path(path_id='')


def test_trajectory_path_dto_negative_blend_radius_normalised():
    dto = _make_path(blend_radius=-0.5)
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_zero_blend_radius_unchanged():
    dto = _make_path(blend_radius=0.0)
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_positive_blend_radius_unchanged():
    dto = _make_path(blend_radius=0.3)
    assert dto.blend_radius == 0.3


def test_trajectory_path_dto_frozen():
    dto = _make_path()
    with pytest.raises((ValidationError, TypeError)):
        dto.path_id = 'new-id'


def test_trajectory_goal_dto_empty_paths():
    with pytest.raises(ValidationError):
        TrajectoryGoalDTO(paths=[])


def test_trajectory_goal_dto_valid():
    path = _make_path()
    dto = TrajectoryGoalDTO(paths=[path])
    assert len(dto.paths) == 1
