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

import queue
import threading
import time
from collections.abc import Iterator

from moveit.planning import MoveItPy, PlanRequestParameters, PlanningComponent
from moveit.core.robot_state import robotStateToRobotStateMsg
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    MotionSequenceItem,
    MotionSequenceRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionSequence
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

    def __init__(self, moveit: MoveItPy, moveit_group_name: str, node) -> None:
        self._moveit = moveit
        self._group_name = moveit_group_name
        self._node = node
        self._planning_component: PlanningComponent = self._moveit.get_planning_component(moveit_group_name)
        self._plan_seq_client = node.create_client(GetMotionSequence, '/plan_sequence_path')
        self._plan_queue: queue.Queue | None = None
        self._cancel_event: threading.Event | None = None
        self._planning_thread: threading.Thread | None = None

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Return True if /plan_sequence_path is available within timeout_sec."""
        return self._plan_seq_client.wait_for_service(timeout_sec=timeout_sec)

    # region: Phase 4 — look-ahead planning

    def plan_all(self, groups: list[list[TrajectoryPathDTO]]) -> None:
        """Start a background daemon thread that plans all groups sequentially (D-04).

        Creates a fresh queue.Queue and threading.Event per call.
        The thread pushes PlanResultDTO items for each group, then a StopIteration sentinel.
        Call iterate_planned_trajectories() to consume results as they become available.
        """
        self._cancel_event = threading.Event()
        self._plan_queue = queue.Queue()
        self._planning_thread = threading.Thread(
            target=self._planning_loop, args=(groups,), daemon=True
        )
        self._planning_thread.start()

    def _planning_loop(self, groups: list[list[TrajectoryPathDTO]]) -> None:
        """Background thread body: plans groups sequentially, propagates end-state (D-08)."""
        # Get current robot state once before any group (Pitfall 7: not per-group)
        with self._moveit.get_planning_scene_monitor().read_only() as scene:
            last_predicted_state: RobotState = robotStateToRobotStateMsg(scene.current_state)

        for group in groups:
            if self._cancel_event.is_set():
                break

            result = self._plan_group_sequence(group, last_predicted_state)

            if self._cancel_event.is_set():
                break

            if result is None:
                # Planning failed — push failure DTO then sentinel and stop
                path_ids = [p.path_id for p in group]
                self._plan_queue.put(PlanResultDTO(
                    success=False,
                    path_ids=path_ids,
                    error_message=f'Sequence planning failed for paths: {path_ids}',
                ))
                self._plan_queue.put(StopIteration)
                return

            # Propagate end-state for next group (D-08)
            last_predicted_state = self._extract_end_state(result.trajectories)
            self._plan_queue.put(result)

        # Normal completion or cancel break
        self._plan_queue.put(StopIteration)

    def _plan_group_sequence(
        self,
        group: list[TrajectoryPathDTO],
        start_state_msg: RobotState,
    ) -> PlanResultDTO | None:
        """Plan a group of paths as a MotionSequenceRequest via /plan_sequence_path.

        Returns PlanResultDTO on success, None on failure or cancellation.
        """
        seq_req = MotionSequenceRequest()
        for i, path_dto in enumerate(group):
            item = MotionSequenceItem()
            # PILZ constraint: last item in group MUST have blend_radius=0.0
            item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
            item.req.group_name = self._group_name
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', or 'CIRC'
            item.req.allowed_planning_time = 5.0
            item.req.max_velocity_scaling_factor = 0.1
            item.req.max_acceleration_scaling_factor = 0.1
            item.req.goal_constraints = [
                self._build_pose_goal_constraints(
                    path_dto.tool_frame or 'tool0', path_dto.target_pose
                )
            ]
            if i == 0:
                item.req.start_state = start_state_msg
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                item.req.path_constraints = self._build_circ_constraints(path_dto)
            seq_req.items.append(item)

        request = GetMotionSequence.Request()
        request.request = seq_req

        future = self._plan_seq_client.call_async(request)

        # Poll until done, checking cancel flag
        while not future.done():
            if self._cancel_event.is_set():
                return None
            time.sleep(0.005)

        response = future.result()
        if response is None or response.response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        trajectories = list(response.response.planned_trajectories)
        return PlanResultDTO(
            success=True,
            trajectories=trajectories,
            path_ids=[p.path_id for p in group],
            blended=len(group) > 1,
        )

    @staticmethod
    def _build_pose_goal_constraints(link_name: str, pose_stamped) -> Constraints:
        """Build goal Constraints from a PoseStamped (replaces C++ constructGoalConstraints)."""
        constraints = Constraints()

        pos = PositionConstraint()
        pos.header = pose_stamped.header
        pos.link_name = link_name
        pos.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.0001]
        target = Pose()
        target.position = pose_stamped.pose.position
        target.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target]
        pos.constraint_region = bv
        constraints.position_constraints = [pos]

        ori = OrientationConstraint()
        ori.header = pose_stamped.header
        ori.link_name = link_name
        ori.orientation = pose_stamped.pose.orientation
        ori.absolute_x_axis_tolerance = 0.001
        ori.absolute_y_axis_tolerance = 0.001
        ori.absolute_z_axis_tolerance = 0.001
        ori.weight = 1.0
        constraints.orientation_constraints = [ori]

        return constraints

    @staticmethod
    def _extract_end_state(trajectories: list) -> RobotState:
        """Extract the final joint state from the last planned trajectory (D-08)."""
        last_traj = trajectories[-1]
        jt = last_traj.joint_trajectory
        last_point = jt.points[-1]
        state = RobotState()
        state.joint_state.name = list(jt.joint_names)
        state.joint_state.position = list(last_point.positions)
        state.joint_state.velocity = [0.0] * len(jt.joint_names)
        return state

    def iterate_planned_trajectories(self) -> Iterator[PlanResultDTO]:
        """Generator that yields PlanResultDTO per group, blocking until each is ready (D-05).

        Terminates when the background thread pushes the StopIteration sentinel.
        """
        while True:
            item = self._plan_queue.get()
            if item is StopIteration:
                return
            yield item

    def cancel(self) -> None:
        """Non-blocking cancellation: set cancel flag, drain queue, push StopIteration sentinel (D-09).

        Idempotent — safe to call before plan_all() or multiple times.
        Does NOT join the planning thread.
        """
        if self._cancel_event is not None:
            self._cancel_event.set()
        if self._plan_queue is not None:
            with self._plan_queue.mutex:
                self._plan_queue.queue.clear()
            self._plan_queue.put(StopIteration)

    # endregion: Phase 4 — look-ahead planning

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
