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

from queue import Queue
from threading import Event
from asyncio import Future
from collections.abc import Iterator

from rclpy.lifecycle.node import LifecycleNode
from rclpy.client import Client
from rclpy import Future as RosFuture
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionSequenceResponse,
    MotionSequenceItem,
    MotionSequenceRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    PlanningSceneComponents,
    RobotTrajectory
)
from moveit_msgs.srv import GetMotionSequence, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.exceptions import (
    AbortPlanningError,
    NotInitializedError
)
from movement_controller.models import (
    ConstraintConfigDTO,
    PlanResultDTO,
    PlanningSessionDTO,
    TrajectoryPathDTO
)


class PilzPlannerService:
    """Wraps MoveItPy to plan a single path via the PILZ industrial motion planner.

    Receives ``moveit`` (MoveItPy instance) and ``moveit_group_name`` via constructor
    injection. The owning controller manages the MoveItPy lifecycle.
    """

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        self._group_name = moveit_group_name
        self._node = node
        self._logger = node.get_logger()
        self._plan_seq_client: Client | None = None
        self._scene_monitor_client: Client | None = None
        self._plan_queue: Queue[PlanResultDTO|AbortPlanningError|StopIteration] | None = None
        self._cancel_event: Event | None = None
        self._planning_session: PlanningSessionDTO | None = None
        self._constraint_config: ConstraintConfigDTO | None = None

        self._callback_group = ReentrantCallbackGroup()

    # region: private methods
    def _build_pose_goal_constraints(self, link_name: str, pose_stamped) -> Constraints:
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

    def _build_path_constraints(self, tool_frame: str) -> Constraints:
        """Build path Constraints from the active constraint config for a given tool frame."""
        constraints = Constraints()
        if self._constraint_config is None:
            return constraints
        cfg = self._constraint_config

        # Workspace BOX (when workspace bounds are tighter than sentinel range)
        if cfg.workspace_enabled:
            pos = PositionConstraint()
            pos.header.frame_id = 'base_link'
            pos.link_name = tool_frame
            pos.weight = 1.0
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [cfg.x_max - cfg.x_min, cfg.y_max - cfg.y_min, cfg.z_max - cfg.z_min]
            center = Pose()
            center.position.x = (cfg.x_max + cfg.x_min) / 2.0
            center.position.y = (cfg.y_max + cfg.y_min) / 2.0
            center.position.z = (cfg.z_max + cfg.z_min) / 2.0
            center.orientation.w = 1.0
            bv = BoundingVolume()
            bv.primitives = [box]
            bv.primitive_poses = [center]
            pos.constraint_region = bv
            constraints.position_constraints = [pos]

        # Joint constraints (midpoint with symmetric tolerances)
        if cfg.joint_constraints_enabled:
            joint_constraints = []
            for name, lower, upper in zip(
                cfg.joint_names, cfg.joint_lower_limits, cfg.joint_upper_limits
            ):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = (lower + upper) / 2.0
                jc.tolerance_above = upper - jc.position
                jc.tolerance_below = jc.position - lower
                jc.weight = 1.0
                joint_constraints.append(jc)
            constraints.joint_constraints = joint_constraints

        # Orientation constraint (identity quaternion as neutral reference)
        if cfg.orientation_constraint_enabled:
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = tool_frame
            oc.orientation.w = 1.0
            oc.absolute_x_axis_tolerance = cfg.orientation_tolerance_x
            oc.absolute_y_axis_tolerance = cfg.orientation_tolerance_y
            oc.absolute_z_axis_tolerance = cfg.orientation_tolerance_z
            oc.parameterization = 0
            oc.weight = 1.0
            constraints.orientation_constraints = [oc]

        return constraints

    def _merge_circ_and_path_constraints(self, circ: Constraints, path: Constraints) -> Constraints:
        """Merge CIRC arc central point path constraint with general path constraints

        Preserves the CIRC arc point at position_constraints[0] and appends workspace BOX
        (if any) from path constraints at [1:]. This avoids overwriting the arc definition
        that PILZ reads for CIRC planning.
        """
        merged = Constraints()
        merged.name = circ.name
        merged.position_constraints = [circ.position_constraints[0]] + path.position_constraints  # type: ignore
        merged.joint_constraints = path.joint_constraints
        merged.orientation_constraints = path.orientation_constraints
        return merged

    def _build_circ_constraints(self, path_dto: TrajectoryPathDTO) -> Constraints:
        """Build path constraints for PILZ CIRC planner."""
        constraints = Constraints()
        constraints.name = path_dto.circ_type.value  # 'interim' or 'center'

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = path_dto.target_pose.header.frame_id
        pos_constraint.link_name = path_dto.tool_frame

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

    def _extract_end_state(self, motion_sequence_response: MotionSequenceResponse) -> RobotState:
        """Extract the final joint state from the last planned trajectory (D-08)."""
        trajectories: list[RobotTrajectory] = motion_sequence_response.planned_trajectories  # type: ignore
        last_traj = trajectories[-1]
        jt = last_traj.joint_trajectory
        last_point = jt.points[-1]  # type: ignore
        state = RobotState()
        state.joint_state.name = list(jt.joint_names)
        state.joint_state.position = list(last_point.positions)
        state.joint_state.velocity = [0.0] * len(jt.joint_names)
        return state
    
    def _generate_motion_sequence_request(self, group: list[TrajectoryPathDTO], start_state_msg: RobotState) -> MotionSequenceRequest:
        """Helper method to generate a MotionSequenceRequest from a group of TrajectoryPathDTOs and a start state."""
        items: list[MotionSequenceItem] = []
        for i, path_dto in enumerate(group):
            item = MotionSequenceItem()
            # PILZ constraint: last item in group MUST have blend_radius=0.0
            item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
            item.req.group_name = self._group_name
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', or 'CIRC'
            item.req.allowed_planning_time = 5.0
            item.req.max_velocity_scaling_factor = 1.0  # this is joint-level scaling; we use cartesian speed limits for LIN/CIRC, so we set this to 1.0 to avoid unintended additional scaling
            item.req.max_acceleration_scaling_factor = 1.0
            item.req.goal_constraints = [
                self._build_pose_goal_constraints(
                    path_dto.tool_frame, path_dto.target_pose
                )
            ]
            if path_dto.motion_type in [MotionTypeEnum.LIN, MotionTypeEnum.CIRC]:
                item.req.cartesian_speed_limited_link = path_dto.tool_frame
                # if cartesian speed is set (non-zero), use it; otherwise, leave at 0.0 so we use max allowed speed
                # NOTE: this requires that we are using 'default_planner_request_adapters/LimitMaxCartesianLinkSpeed'
                # in robot moveit config!
                item.req.max_cartesian_speed = path_dto.cartesian_speed if path_dto.cartesian_speed > 0.0 else 0.0

            if i == 0:
                item.req.start_state = start_state_msg

            path_constraints = self._build_path_constraints(path_dto.tool_frame)
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                circ_constraints = self._build_circ_constraints(path_dto)
                item.req.path_constraints = self._merge_circ_and_path_constraints(
                    circ_constraints, path_constraints
                )
            else:
                item.req.path_constraints = path_constraints

            items.append(item)
        
        seq_req = MotionSequenceRequest()
        seq_req.items = items
        return seq_req
    
    # endregion: private methods

    # region: callbacks
    def _push_and_plan_next(self, future: Future[GetMotionSequence.Response]) -> None:
        """Callback for GetMotionSequence future; pushes result to queue and starts next plan if not done."""
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning sequence cancelled; skipping result processing and not scheduling next plan')
            return
        
        try:
            response = future.result()
            if response is None or response.response.error_code.val != MoveItErrorCodes.SUCCESS:
                err_msg = 'planning sequence service call failed or returned error code; aborting planning thread'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(AbortPlanningError(err_msg))
                return
            path_ids = (
                [p.path_id for p in self._planning_session.current_group] if self._planning_session 
                else []
            ) 
            if self._plan_queue is not None:
                self._plan_queue.put(
                    PlanResultDTO(
                        success=True,
                        motion_plan=response.response,
                        path_ids=path_ids,
                        blended=len(path_ids) > 1,
                    )
                )
            # schedule next plan if more groups remain
            path_group = self._planning_session.get_next_group() if self._planning_session else None
            if path_group:
                request = GetMotionSequence.Request()
                request.request = self._generate_motion_sequence_request(
                    path_group,
                    self._extract_end_state(response.response)
                )
                if self._plan_seq_client is not None:  # we don't wait for service; we assume it is still available
                    plan_future: RosFuture = self._plan_seq_client.call_async(request)
                    plan_future.add_done_callback(self._push_and_plan_next)
                else:
                    err_msg = 'planning sequence service client not available; cannot continue planning thread'
                    self._logger.error(err_msg)
                    if self._plan_queue is not None:
                        self._plan_queue.put(AbortPlanningError(err_msg))
            else:
                self._logger.info('all paths planned successfully; finishing planning thread')
                if self._plan_queue is not None:
                    self._plan_queue.put(StopIteration())
        except Exception as e:
            err_msg = f'exception while processing planning sequence response {e}'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(AbortPlanningError(err_msg))

    def _initiate_planning(self, future: Future[GetPlanningScene.Response]) -> None:
        """Callback for GetPlanningScene future; starts the planning thread if scene retrieval succeeded."""
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning sequence cancelled; skipping planning scheduling')
            return
        
        try:
            response = future.result()
            if response is None or response.scene.robot_state is None:
                err_msg = 'failed to retrieve planning scene or robot state; aborting planning'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(AbortPlanningError(err_msg))
                return
            
            path_group = self._planning_session.get_next_group() if self._planning_session else None
            if path_group is None:
                err_msg = 'no groups to plan; aborting planning'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(AbortPlanningError(err_msg))
                return
            
            # schedule planning for first group of trajectories
            robot_state_msg = response.scene.robot_state
            request = GetMotionSequence.Request()
            request.request = self._generate_motion_sequence_request(path_group, robot_state_msg)
            if (
                self._plan_seq_client is not None and
                self._plan_seq_client.wait_for_service(timeout_sec=5.0)  # first time we wait for service
            ):
                plan_future: RosFuture = self._plan_seq_client.call_async(request)
                plan_future.add_done_callback(self._push_and_plan_next)
            else:
                err_msg = 'planning sequence service not available; cannot start planning thread'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(AbortPlanningError(err_msg))
                return
        except Exception as e:
            err_msg = f'exception while retrieving planning scene: {e}'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(AbortPlanningError(err_msg))

    # endregion: callbacks

    # region: public methods
    def set_constraints(self, dto: ConstraintConfigDTO) -> None:
        """Store validated constraint config"""
        self._constraint_config = dto

    def on_activate(self) -> None:
        self._plan_seq_client = self._node.create_client(
            srv_type=GetMotionSequence,
            srv_name='/move_group/plan_sequence_path',
            callback_group=self._callback_group
        )
        self._scene_monitor_client = self._node.create_client(
            srv_type=GetPlanningScene,
            srv_name='/move_group/get_planning_scene',
            callback_group=self._callback_group
        )
        self._cancel_event = None
        self._plan_queue = None

    def on_deactivate(self) -> None:
        if self._plan_seq_client is not None:
            self._node.destroy_client(self._plan_seq_client)
            self._plan_seq_client = None
        if self._scene_monitor_client is not None:
            self._node.destroy_client(self._scene_monitor_client)
            self._scene_monitor_client = None
        self._planning_session = None

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Return True if /plan_sequence_path is available within timeout_sec."""
        if self._plan_seq_client is None:
            self._logger.error('wait_for_service called before activation')
            return False
        return self._plan_seq_client.wait_for_service(timeout_sec=timeout_sec)

    def plan_all(self, groups: list[list[TrajectoryPathDTO]]) -> bool:
        """Start a background daemon thread that plans all groups sequentially (D-04).

        Creates a fresh queue.Queue and threading.Event per call.
        The thread pushes PlanResultDTO items for each group, then a StopIteration sentinel.
        Call iterate_planned_trajectories() to consume results as they become available.
        """
        self._cancel_event = Event()
        self._plan_queue = Queue()
        self._planning_session = PlanningSessionDTO(groups=groups)
        
        if (
            self._scene_monitor_client is not None and 
            self._scene_monitor_client.wait_for_service(timeout_sec=5.0)
        ):
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            future: RosFuture = self._scene_monitor_client.call_async(request)
            future.add_done_callback(self._initiate_planning)
            
            return True
        else:
            self._logger.error('planning scene service not available; cannot start planning thread')
            self._cancel_event = None
            self._plan_queue = None
            self._planning_session = None

            return False

    def iterate_planned_trajectories(self) -> Iterator[PlanResultDTO]:
        """Generator that yields PlanResultDTO per group, blocking until each is ready (D-05).

        Terminates when the background thread pushes the StopIteration sentinel.
        """
        while True:
            if self._plan_queue:
                item = self._plan_queue.get()
                if isinstance(item, StopIteration):
                    self._plan_queue = None
                    self._cancel_event = None
                    self._planning_session = None
                    return
                elif isinstance(item, AbortPlanningError):
                    self._plan_queue = None
                    self._cancel_event = None
                    self._planning_session = None
                    raise item

                yield item
            else:
                raise NotInitializedError('iterate_planned_trajectories called before plan_all')

    def cancel(self) -> None:
        """Non-blocking cancellation: set cancel flag, drain queue, push StopIteration sentinel (D-09).

        Idempotent — safe to call before plan_all() or multiple times.
        Does NOT join the planning thread.
        """
        if self._cancel_event is not None and not self._cancel_event.is_set():
            self._cancel_event.set()
        if self._plan_queue is not None:
            with self._plan_queue.mutex:
                self._plan_queue.queue.clear()
            self._plan_queue.put(AbortPlanningError('planning cancelled by requester'))

    # endregion: public methods
