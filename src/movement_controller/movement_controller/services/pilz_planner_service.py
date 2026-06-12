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
    InvalidStartStateError,
)
from movement_controller.models import (
    ConstraintConfigDTO,
    PlanResultDTO,
    PlanningSessionDTO,
    TrajectoryPathDTO
)


class PilzPlannerService:
    """Implements a planning service that uses the PILZ industrial motion planner to plan a 
    sequence of TrajectoryPathDTOs.
    """

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        self._group_name = moveit_group_name
        self._node = node
        self._logger = node.get_logger()
        self._plan_seq_client: Client | None = None
        self._scene_monitor_client: Client | None = None
        self._plan_queue: Queue[PlanResultDTO|StopIteration] | None = None
        self._cancel_event: Event | None = None
        self._planning_session: PlanningSessionDTO | None = None
        self._constraint_config: ConstraintConfigDTO | None = None

        self._callback_group = ReentrantCallbackGroup()

    # region: private methods
    def _build_pose_goal_constraints(self, link_name: str, pose_stamped) -> Constraints:
        """Build goal Constraints from a PoseStamped (replaces C++ constructGoalConstraints)."""
        constraints = Constraints()
        constraints.name = f'goal_constraints_{link_name}'

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

        self._logger.debug(f'Constructed goal constraints for link {link_name}:\n{constraints}')

        return constraints

    def _build_path_constraints(self, tool_frame: str) -> Constraints:
        """Build path Constraints from the active constraint config for a given tool frame."""
        constraints = Constraints()
        constraints.name = f'path_constraints_{tool_frame}'
        if self._constraint_config is None:
            return constraints
        self._logger.debug(f'Building path constraints')
        cfg = self._constraint_config

        # Workspace BOX (when workspace bounds are tighter than sentinel range)
        if cfg.workspace_enabled:
            self._logger.debug(
                f'Building workspace constraints with bounds x: [{cfg.x_min}, {cfg.x_max}], '
                f'y: [{cfg.y_min}, {cfg.y_max}], z: [{cfg.z_min}, {cfg.z_max}]'
            )  # FIXME: we should use workspace constraint instead
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
            constraints.position_constraints.append(pos)  # type: ignore
            self._logger.debug(f'Constructed workspace constraints:\n{constraints.position_constraints}')

        # Joint constraints (midpoint with symmetric tolerances)
        if cfg.joint_constraints_enabled:
            self._logger.debug(
                f'Building joint constraints for joints {cfg.joint_names} '
                f'with lower limits {cfg.joint_lower_limits} and upper '
                f'limits {cfg.joint_upper_limits}'
            )
            for name, lower, upper in zip(
                cfg.joint_names, cfg.joint_lower_limits, cfg.joint_upper_limits
            ):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = (lower + upper) / 2.0
                jc.tolerance_above = upper - jc.position
                jc.tolerance_below = jc.position - lower
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)  # type: ignore
            self._logger.debug(f'Constructed joint constraints:\n{constraints.joint_constraints}')

        # Orientation constraint (identity quaternion as neutral reference)
        if cfg.orientation_constraint_enabled:
            self._logger.debug(
                f'Building orientation constraints with tolerances x: {cfg.orientation_tolerance_x}, '
                f'y: {cfg.orientation_tolerance_y}, z: {cfg.orientation_tolerance_z} for tool frame {tool_frame}'
            )
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = tool_frame
            oc.orientation.w = 1.0
            oc.absolute_x_axis_tolerance = cfg.orientation_tolerance_x
            oc.absolute_y_axis_tolerance = cfg.orientation_tolerance_y
            oc.absolute_z_axis_tolerance = cfg.orientation_tolerance_z
            oc.parameterization = 0
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)  # type: ignore
            self._logger.debug(f'Constructed orientation constraints:\n{constraints.orientation_constraints}')

        self._logger.debug(f'Final constructed path constraints:\n{constraints}')
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
        self._logger.debug(f'Merged CIRC and path constraints:\n{merged}')
        return merged

    def _build_circ_constraints(self, path_dto: TrajectoryPathDTO) -> Constraints:
        """Build path constraints for PILZ CIRC planner."""
        constraints = Constraints()
        constraints.name = path_dto.circ_type.value  # 'interim' or 'center'

        pos_constraint = PositionConstraint()
        pos_constraint.header = path_dto.target_pose.header
        pos_constraint.link_name = path_dto.tool_frame
        pos_constraint.weight = 0.1

        point_pose = Pose()
        point_pose.position.x = path_dto.circ_point.x
        point_pose.position.y = path_dto.circ_point.y
        point_pose.position.z = path_dto.circ_point.z
        point_pose.orientation.w = 1.0

        bv = BoundingVolume()
        bv.primitives = []
        bv.primitive_poses = [point_pose]
        pos_constraint.constraint_region = bv

        constraints.position_constraints = [pos_constraint]
        self._logger.debug(
            f'Constructed CIRC constraints for path {path_dto.path_id}:\n'
            f'{constraints.position_constraints}'
        )
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
    
    def _start_state_invalid(self, start_state: RobotState) -> bool:
        """Check if the start state violates any active constraints """
        if self._constraint_config is None:
            return False
        
        self._logger.debug(f'Checking if start state is valid:\n{start_state}')
        
        cfg = self._constraint_config

        # Check workspace bounds
        if cfg.workspace_enabled:
            self._logger.debug(f'Checking workspace bounds violations')
            # TODO: to check workspace bounds
        
        # Check joint limits
        if cfg.joint_constraints_enabled:
            self._logger.debug(f'Checking joint constraints violations')
            for name, lower, upper in zip(
                cfg.joint_names, cfg.joint_lower_limits, cfg.joint_upper_limits
            ):
                if name in start_state.joint_state.name:
                    idx = start_state.joint_state.name.index(name)  # type: ignore
                    pos = start_state.joint_state.position[idx]  # type: ignore
                    if pos < lower or pos > upper:
                        self._logger.debug(
                            f'Start state joint {name} with position {pos} violates limits [{lower}, {upper}]'
                        )
                        return True
        
        # Check orientation constraints
        if cfg.orientation_constraint_enabled:
            self._logger.debug(f'Checking orientation constraints violations')
            # TODO: to check orientation constraints

        return False
    
    def _calculate_scaling_factors(self, path_dto: TrajectoryPathDTO) -> tuple[float, float]:
        """Calculate velocity and acceleration scaling factors based on cartesian speed limit for LIN/CIRC paths."""
        if self._constraint_config is None:
            self._logger.warning('Constraint config not set; using default scaling factors of 1.0')
            return 1.0, 1.0
        
        if path_dto.motion_type in [MotionTypeEnum.LIN, MotionTypeEnum.CIRC]:
            if self._constraint_config.max_cartesian_speed > 0.0:
                self._logger.debug("Max cartesian speed is set, calculating scaling factor")
                vel_scaling_factor = min(
                    1.0,
                    path_dto.cartesian_speed / self._constraint_config.max_cartesian_speed
                )
            else:
                vel_scaling_factor = 1.0
            
            if self._constraint_config.max_cartesian_acceleration > 0.0:
                self._logger.debug("Max cartesian acceleration is set, calculating scaling factor")
                acc_scaling_factor = min(
                    1.0,
                    path_dto.cartesian_acceleration / self._constraint_config.max_cartesian_acceleration
                )
            else:
                acc_scaling_factor = 1.0
            
            self._logger.debug(
                f'Calculated scaling factors for path {path_dto.path_id} with cartesian speed {path_dto.cartesian_speed}: '
                f'velocity_scaling_factor={vel_scaling_factor}, acceleration_scaling_factor={acc_scaling_factor}'
            )
            return vel_scaling_factor, acc_scaling_factor
        else:
            if self._constraint_config.max_joint_speed > 0.0:
                self._logger.debug("Max joint speed is set, calculating scaling factor")
                vel_scaling_factor = min(
                    1.0,
                    path_dto.joint_speed / self._constraint_config.max_joint_speed
                )
            else:
                vel_scaling_factor = 1.0
            
            if self._constraint_config.max_joint_acceleration > 0.0:
                self._logger.debug("Max joint acceleration is set, calculating scaling factor")
                acc_scaling_factor = min(
                    1.0,
                    path_dto.joint_acceleration / self._constraint_config.max_joint_acceleration
                )
            else:
                acc_scaling_factor = 1.0
            
            self._logger.debug(
                f'Calculated scaling factors for path {path_dto.path_id} with joint speed {path_dto.joint_speed}: '
                f'velocity_scaling_factor={vel_scaling_factor}, acceleration_scaling_factor={acc_scaling_factor}'
            )
            return vel_scaling_factor, acc_scaling_factor
    
    def _generate_motion_sequence_request(self, group: list[TrajectoryPathDTO], start_state_msg: RobotState) -> MotionSequenceRequest:
        """Helper method to generate a MotionSequenceRequest from a group of TrajectoryPathDTOs and a start state."""
        self._logger.debug(f'Generating MotionSequenceRequest for group of {len(group)} paths with start state:\n{start_state_msg}')
        
        if self._start_state_invalid(start_state_msg):
            err_msg = 'invalid start state - violation of constraints'
            self._logger.error(err_msg)
            raise InvalidStartStateError(err_msg)

        items: list[MotionSequenceItem] = []
        for i, path_dto in enumerate(group):
            self._logger.debug(f'Generating motion sequence item for path {path_dto.path_id}')
            item = MotionSequenceItem()
            # PILZ constraint: last item in group MUST have blend_radius=0.0
            item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
            item.req.group_name = self._group_name
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', or 'CIRC'
            item.req.allowed_planning_time = 5.0
            item.req.num_planning_attempts = 1
            (
                item.req.max_velocity_scaling_factor,
                item.req.max_acceleration_scaling_factor
            ) = self._calculate_scaling_factors(path_dto)
            item.req.goal_constraints = [
                self._build_pose_goal_constraints(
                    path_dto.tool_frame, path_dto.target_pose
                )
            ]
            
            if i == 0:
                self._logger.debug(
                    f'Setting start state for first path in group:\n{start_state_msg}'
                )
                item.req.start_state = start_state_msg

            path_constraints = self._build_path_constraints(path_dto.tool_frame)
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                self._logger.debug(
                    f'Building CIRC-specific constraints for path {path_dto.path_id} with CIRC '
                    f'type {path_dto.circ_type}'
                )
                circ_constraints = self._build_circ_constraints(path_dto)
                item.req.path_constraints = self._merge_circ_and_path_constraints(
                    circ_constraints, path_constraints
                )
            else:
                item.req.path_constraints = path_constraints
            
            self._logger.debug(f'Final path constraint:\n{item.req.path_constraints}')

            items.append(item)
        
        self._logger.debug(f'Final list of items into MotionSequenceRequest:\n{items}')

        seq_req = MotionSequenceRequest()
        seq_req.items = items
        return seq_req
    
    # endregion: private methods

    # region: callbacks
    def _push_and_plan_next(self, future: Future[GetMotionSequence.Response]) -> None:
        """Callback for GetMotionSequence future; pushes result to queue and starts next plan if not done."""
        self._logger.debug('Received motion sequence response - processing and scheduling next')
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning sequence cancelled; skipping result processing and not scheduling next plan')
            return
        
        try:
            response = future.result()
            if response is None or response.response.error_code.val != MoveItErrorCodes.SUCCESS:
                err_msg = f'planning sequence service call failed with error code {response.response.error_code.val}; aborting planning thread'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(PlanResultDTO(error_message=err_msg))
                return
            path_ids = (
                [p.path_id for p in self._planning_session.current_group] if self._planning_session 
                else []
            )
            self._logger.debug(
                f'Planned motion sequence response for path IDs {path_ids}:\n{response.response}'
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
            self._logger.debug(f'Next path group to plan: {path_group}')
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
                        self._plan_queue.put(PlanResultDTO(error_message=err_msg))
            else:
                self._logger.info('all paths planned successfully; finishing planning thread')
                if self._plan_queue is not None:
                    self._plan_queue.put(StopIteration())
        except InvalidStartStateError as e:
            err_msg = f'robot start state violates one of the constraints; aborting planning sequence'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(PlanResultDTO(error_message=err_msg))
        except Exception as e:
            err_msg = f'exception while processing planning sequence response - {e}'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(PlanResultDTO(error_message=err_msg))

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
                    self._plan_queue.put(PlanResultDTO(error_message=err_msg))
                return
            
            path_group = self._planning_session.get_next_group() if self._planning_session else None
            if path_group is None:
                err_msg = 'no groups to plan; aborting planning'
                self._logger.error(err_msg)
                if self._plan_queue is not None:
                    self._plan_queue.put(PlanResultDTO(error_message=err_msg))
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
                    self._plan_queue.put(PlanResultDTO(error_message=err_msg))
                return
        except InvalidStartStateError as e:
            err_msg = f'robot start state violates one of the constraints; aborting planning sequence'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(PlanResultDTO(error_message=err_msg))
        except Exception as e:
            err_msg = f'exception while retrieving planning scene - {e}'
            self._logger.error(err_msg)
            if self._plan_queue is not None:
                self._plan_queue.put(PlanResultDTO(error_message=err_msg))

    # endregion: callbacks

    # region: public methods
    def set_constraints(self, dto: ConstraintConfigDTO) -> None:
        """Store validated constraint config"""
        self._constraint_config = dto

    def on_activate(self) -> None:
        self._plan_seq_client = self._node.create_client(
            srv_type=GetMotionSequence,
            srv_name='plan_sequence_path',
            callback_group=self._callback_group
        )
        self._scene_monitor_client = self._node.create_client(
            srv_type=GetPlanningScene,
            srv_name='get_planning_scene',
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

                yield item
            else:
                yield PlanResultDTO(error_message='plan queue not initialized; did you call plan_all()?')

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
            self._plan_queue.put(PlanResultDTO(error_message='planning cancelled by requester'))

    # endregion: public methods
