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
"""PilzPlannerService — plans trajectories via the PILZ industrial motion planner.

Stub implementation. Full planner logic is wired in Phase 3 Plan 02.
"""
from __future__ import annotations

from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class PilzPlannerService:
    """Wraps MoveItPy to plan a single path via the PILZ industrial motion planner.

    Receives ``moveit`` (MoveItPy instance) and ``planning_component`` via constructor
    injection. The owning controller manages the MoveItPy lifecycle.
    """

    def __init__(self, moveit: object, planning_component: object) -> None:
        self._moveit = moveit
        self._planning_component = planning_component

    def plan(self, path_dto: TrajectoryPathDTO) -> object:
        """Plan a single trajectory path.

        Returns a plan result object. Raises NotImplementedError until Phase 3 Plan 02.
        """
        raise NotImplementedError(
            'PilzPlannerService.plan() not yet implemented — see Phase 3 Plan 02'
        )
