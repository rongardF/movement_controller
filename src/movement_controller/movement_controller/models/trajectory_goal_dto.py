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
"""TrajectoryGoalDTO — Pydantic DTO wrapping the ExecuteTrajectory goal."""

from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class TrajectoryGoalDTO(BaseModel):
    """Validated, immutable goal containing an ordered list of trajectory paths."""

    model_config = ConfigDict(frozen=True)

    paths: list[TrajectoryPathDTO] = Field(
        description='Non-empty list of trajectory paths to execute'
    )

    @field_validator('paths')  #FIXME: HUMAN REVIEW COMMENT: is it validating before or after? Perhaps we shoudl be explicit about this? This applies here and in other places where we use field validators, it would be good to be consistent and clear about when the validation is happening
    @classmethod
    def validate_paths_non_empty(cls, v: list) -> list:
        if not v:    #FIXME: HUMAN REVIEW COMMENT: I think we should also validate for duplicate path_ids here, to ensure that all paths in the list have unique identifiers. This would help to prevent potential issues during trajectory execution where duplicate path_ids could cause confusion or errors. We could use a set to track seen path_ids and raise a ValueError if we encounter a duplicate. What do you think?
            raise ValueError('paths list must be non-empty')
        return v
    
     #FIXME: HUMAN REVIEW COMMENT: I think it would simplify code if we had a static method 'from_ros_msg' that takes the ROS message and constructs the DTO, instead of having this logic in the controller. It would also help to keep the controller code cleaner and more focused on its main responsibilities. It would internally convert all 'TrajectoryPath' messages to 'TrajectoryPathDTO' instances, so the controller can work with the DTOs directly. What do you think?
