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
"""Unit tests for TrajectoryGrouper blend grouping algorithm."""

import pytest
from geometry_msgs.msg import PoseStamped

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper

# Predefined valid UUID4 values for deterministic tests.
_UUID_A = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID_B = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'
_UUID_C = 'cccccccc-cccc-4ccc-8ccc-cccccccccccc'
_UUID_D = 'dddddddd-dddd-4ddd-8ddd-dddddddddddd'
_UUIDT = [
    '00000000-0000-4000-8000-000000000000',
    '11111111-1111-4111-8111-111111111111',
    '22222222-2222-4222-8222-222222222222',
    '33333333-3333-4333-8333-333333333333',
    '44444444-4444-4444-8444-444444444444',
    '55555555-5555-4555-8555-555555555555',
    '66666666-6666-4666-8666-666666666666',
]


def _p(path_id: str, blend_radius: float) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO for grouper tests."""
    return TrajectoryPathDTO(
        path_id=path_id,
        motion_type=MotionTypeEnum.LIN,
        target_pose=PoseStamped(),
        blend_radius=blend_radius,
    )


def test_empty_paths_raises():
    """group() raises ValueError when the input list is empty."""
    with pytest.raises(ValueError, match='must not be empty'):
        TrajectoryGrouper.group([])


def test_single_path_always_new_group():
    """A single path always produces exactly one group."""
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.5)])
    assert len(groups) == 1
    assert len(groups[0]) == 1


def test_all_zero_blend_radius():
    """Each zero-blend-radius path starts its own group."""
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.0), _p(_UUID_B, 0.0), _p(_UUID_C, 0.0)])
    assert len(groups) == 3


def test_mixed_grouping_d07_example():
    """Test the exact D-07 example: 7 paths → 4 groups with sizes [1, 1, 4, 1]."""
    paths = [
        _p(_UUIDT[0], 0.5), _p(_UUIDT[1], 0.0), _p(_UUIDT[2], 0.0),
        _p(_UUIDT[3], 0.3), _p(_UUIDT[4], 0.3), _p(_UUIDT[5], 0.3),
        _p(_UUIDT[6], 0.0),
    ]
    groups = TrajectoryGrouper.group(paths)
    assert len(groups) == 4
    assert len(groups[0]) == 1   # t0 (first path, always new group)
    assert len(groups[1]) == 1   # t1 (blend_radius=0 → new group)
    assert len(groups[2]) == 4   # t2, t3, t4, t5
    assert len(groups[3]) == 1   # t6 (blend_radius=0 → new group)


def test_first_path_always_starts_new_group_even_with_positive_blend_radius():
    """First path always starts a new group regardless of its blend_radius.

    Subsequent paths with br>0 all merge into that group.
    """
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.5), _p(_UUID_B, 0.3), _p(_UUID_C, 0.3)])
    assert len(groups) == 1
    assert len(groups[0]) == 3


def test_two_separate_groups():
    """br=0 on 3rd path splits into two groups of two."""
    groups = TrajectoryGrouper.group(
        [_p(_UUID_A, 0.0), _p(_UUID_B, 0.3), _p(_UUID_C, 0.0), _p(_UUID_D, 0.3)]
    )
    assert len(groups) == 2
    assert len(groups[0]) == 2   # A, B
    assert len(groups[1]) == 2   # C, D


def test_negative_blend_radius_treated_as_zero():
    """DTO normalises -0.5 → 0.0; first path starts group, second merges due to br>0."""
    # After normalisation: A.blend_radius=0.0, B.blend_radius=0.3
    # A is i=0 → new group; B is i=1 with br=0.3>0 → merges → [[A, B]]
    groups = TrajectoryGrouper.group([_p(_UUID_A, -0.5), _p(_UUID_B, 0.3)])
    assert len(groups) == 1
    assert len(groups[0]) == 2
