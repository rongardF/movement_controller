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
"""PilzPlannerService — maps TrajectoryPathDTO to PILZ planner parameters and executes planning."""

from __future__ import annotations

from moveit.planning import MoveItPy, PlanRequestParameters, PlanningComponent
from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO


class PilzPlannerService:
    """Wraps MoveItPy to plan a single path via the PILZ industrial motion planner.

    Receives ``moveit`` (MoveItPy instance) and ``moveit_group_name`` via constructor
    injection. The owning controller manages the MoveItPy lifecycle.
    """

    def __init__(self, moveit: MoveItPy, moveit_group_name: str) -> None:
        self._moveit = moveit
        self._planning_component: PlanningComponent = self._moveit.get_planning_component(moveit_group_name)

    def plan(self, path_dto: TrajectoryPathDTO) -> PlanResultDTO:
        """Plan a single trajectory path using the PILZ industrial motion planner.

        Sets start state to current robot state on every call (D-08), maps
        ``MotionTypeEnum`` to the appropriate PILZ planner ID (D-07), and for CIRC
        paths sets and clears path constraints in a try/finally block.

        Args:
            path_dto: Validated, immutable trajectory path descriptor.

        Returns:
            ``PlanResultDTO(success=True, trajectory=...)`` on success, or
            ``PlanResultDTO(success=False, error_message=...)`` on failure.
        """
        planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', or 'CIRC'

        self._planning_component.set_start_state_to_current_state()
        self._planning_component.set_goal_state(
            pose_stamped_msg=path_dto.target_pose,
            pose_link=path_dto.tool_frame or 'tool0',
        )

        try:
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                constraints = self._build_circ_constraints(path_dto)
                self._planning_component.set_path_constraints(constraints)
            params = PlanRequestParameters(self._moveit, '')
            params.planner_id = planner_id
            params.planning_pipeline = 'pilz_industrial_motion_planner'
            params.planning_attempts = 3
            params.planning_time = 5.0
            params.max_velocity_scaling_factor = 0.1    # Phase 3: m/s→scaling deferred to Phase 5 (CON-05)
            params.max_acceleration_scaling_factor = 0.1  # Phase 3: m/s²→scaling deferred to Phase 5
            plan_result = self._planning_component.plan(single_plan_parameters=params)
        finally:
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                self._planning_component.set_path_constraints(Constraints())

        if not plan_result:
            return PlanResultDTO(
                success=False,
                error_message=f'PILZ {planner_id} planning failed for path {path_dto.path_id!r}',
            )

        return PlanResultDTO(success=True, trajectory=plan_result.trajectory)

    @staticmethod
    def _build_circ_constraints(path_dto: TrajectoryPathDTO) -> Constraints:
        """Build path constraints for PILZ CIRC planner.

        The ``constraints.name`` field selects the CIRC mode:
        - ``'interim'``: ``circ_point`` is a waypoint on the arc.
        - ``'center'``: ``circ_point`` is the center of the arc.

        Args:
            path_dto: Trajectory path with CIRC-specific fields populated.

        Returns:
            A ``Constraints`` message ready to be passed to ``set_path_constraints``.
        """
        constraints = Constraints()
        constraints.name = path_dto.circ_type.value  # 'interim' or 'center'

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = path_dto.target_pose.header.frame_id
        pos_constraint.link_name = path_dto.tool_frame or 'tool0'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # radius ~0; just a position marker

        point_pose = Pose()
        point_pose.position.x = path_dto.circ_point.x
        point_pose.position.y = path_dto.circ_point.y
        point_pose.position.z = path_dto.circ_point.z
        point_pose.orientation.w = 1.0

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [point_pose]
        pos_constraint.constraint_region = bv

        constraints.position_constraints = [pos_constraint]
        return constraints
