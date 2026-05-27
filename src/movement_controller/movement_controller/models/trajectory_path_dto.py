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
"""TrajectoryPathDTO — Pydantic DTO mirroring TrajectoryPath.msg."""

from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Point, PoseStamped
from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.enums.motion_type_enum import MotionTypeEnum

if TYPE_CHECKING: #FIXME: HUMAN REVIEW COMMENT: what is this for? What does it do?
    pass


class TrajectoryPathDTO(BaseModel):
    """Validated, immutable representation of a single trajectory path."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    path_id: str = Field(description='UUID4 path identifier, must be non-empty')  #FIXME: HUMAN REVIEW COMMENT: I think this should be UUID4 type imported from uuid module, rather than a string. It would ensure that the path_id is always a valid UUID4 and would make it clearer to users that this is the expected format
    motion_type: MotionTypeEnum = Field(description='Motion type: LIN, PTP, or CIRC')
    target_pose: PoseStamped = Field(description='Target end-effector pose with frame_id')
    blend_radius: float = Field(
        default=0.0,
        description='Blend radius in metres; negative values normalised to 0.0',
    )
    cartesian_speed: float = Field(
        default=0.0,
        description='End-effector cartesian speed in m/s',
    )
    acceleration: float = Field(
        default=0.0,
        description='End-effector acceleration in m/s²',
    )
    tool_frame: str = Field(
        default='',
        description='Tool frame override; empty string means use tool0',
    )
    circ_type: str = Field( #FIXME: HUMAN REVIEW COMMENT: I think we could use Enum for this as well, since it has a limited set of valid values (e.g. 'interim' or 'center'). It would provide stronger validation and make it clearer to users what the expected values are. What do you think?
        default='',
        description='CIRC arc point interpretation: interim or center',
    )
    circ_point: Point = Field(
        description='CIRC arc reference point; ignored for LIN/PTP',
    )

    @field_validator('path_id')
    @classmethod
    def validate_path_id(cls, v: str) -> str:
        if not v: #FIXME: HUMAN REVIEW COMMENT: should we also validate that it's a valid UUID4 format? We could use the uuid module to attempt to parse it and ensure it's a valid UUID4, which would provide stronger validation than just checking for non-empty string. What do you think?
            raise ValueError('path_id must be non-empty')
        return v

    @field_validator('blend_radius', mode='before')
    @classmethod
    def normalise_blend_radius(cls, v: float) -> float:
        return 0.0 if float(v) < 0 else float(v) #FIXME: HUMAN REVIEW COMMENT: we should account for failure where the value is not parsable to a float, to avoid unexpected exceptions. We could catch the exception and raise a ValueError with a clear message indicating that blend_radius must be a number. What do you think?

    @classmethod
    def from_ros_msg(cls, ros_msg) -> 'TrajectoryPathDTO':
        """Construct a TrajectoryPathDTO from a TrajectoryPath ROS2 message."""
        return cls(
            path_id=ros_msg.path_id, 
            motion_type=ros_msg.motion_type, #FIXME: HUMAN REVIEW COMMENT: should we not cast it to enum here?
            target_pose=ros_msg.target_pose,
            blend_radius=ros_msg.blend_radius,
            cartesian_speed=ros_msg.cartesian_speed,
            acceleration=ros_msg.acceleration,
            tool_frame=ros_msg.tool_frame,
            circ_type=ros_msg.circ_type,
            circ_point=ros_msg.circ_point,
        )
