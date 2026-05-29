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
"""URMovementController — ROS2 LifecycleNode for UR robot trajectory execution."""

from threading import Lock

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from moveit.planning import MoveItPy
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.srv import GetPlanningScene

from movement_controller.action import ExecuteTrajectory
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper


class URMovementController(LifecycleNode):
    """LifecycleNode that exposes an ExecuteTrajectory action server for UR robots."""

    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._is_active: bool = False
        self._is_executing: bool = False
        self._executing_lock: Lock = Lock()
        self._moveit: MoveItPy | None = None
        self._planner_service: PilzPlannerService | None = None

        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name (used from Phase 3 onward)'),
        )
        self.declare_parameter(
            'moveit_connection_timeout',
            10.0,
            ParameterDescriptor(description='Seconds to wait for MoveItPy to connect before failing on_configure'),
        )

    # region: lifecycle callbacks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')

        moveit_group_name = self.get_parameter('moveit_group_name').get_parameter_value().string_value
        timeout: float = self.get_parameter('moveit_connection_timeout').get_parameter_value().double_value

        client = self.create_client(GetPlanningScene, '/move_group/get_planning_scene')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'move_group not available after {timeout}s — is move_group running?'
            )
            self.destroy_client(client)
            return TransitionCallbackReturn.FAILURE
        self.destroy_client(client)

        try:
            self._moveit = MoveItPy(node_name='moveit_py_node')
            self._planner_service = PilzPlannerService(self._moveit, moveit_group_name, node=self)
        except Exception as e:
            self.get_logger().error(f'MoveItPy initialisation failed: {e}')
            if self._moveit is not None:
                self._moveit.shutdown()
                self._moveit = None
            return TransitionCallbackReturn.FAILURE

        if not self._planner_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'/plan_sequence_path service not available after {timeout}s — '
                'is move_group launched with pilz_industrial_motion_planner/MoveGroupSequenceService capability?'
            )
            self._moveit.shutdown()
            self._moveit = None
            self._planner_service = None
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'MoveItPy connected - planning group: {moveit_group_name}')

        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            "movement_controller/execute_trajectory",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f'Action server created at: movement_controller/execute_trajectory')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        self._is_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        self._is_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._moveit is not None:
            self._moveit.shutdown()
            self._moveit = None
        self._planner_service = None
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle callbacks

    # region: action server callbacks

    def _goal_callback(self, goal: ExecuteTrajectory.Goal) -> GoalResponse:
        # 1. Node must be active to accept goals
        if not self._is_active:
            self.get_logger().error('Goal rejected: node is not active')
            return GoalResponse.REJECT

        # 2. Only one goal at a time — check and set atomically to prevent TOCTOU race
        with self._executing_lock:
            if self._is_executing:
                self.get_logger().error('Goal rejected: another goal is already executing')
                return GoalResponse.REJECT
            self._is_executing = True

        # 3. Full goal validation via DTO — reset executing flag on failure
        try:
            TrajectoryGoalDTO.from_ros_msg(goal)
        except (ValidationError, ValueError) as e:
            self.get_logger().error(f'Goal rejected: {e}')
            with self._executing_lock:
                self._is_executing = False
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    async def _execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteTrajectory.Result:
        try:
            goal_dto = TrajectoryGoalDTO.from_ros_msg(goal_handle.request)
            groups = TrajectoryGrouper.group(goal_dto.paths)
            completed_ids: list[str] = []

            if self._planner_service is None or self._moveit is None:
                err = 'Planner service or MoveItPy not initialised'
                self.get_logger().error(err)
                result = ExecuteTrajectory.Result()
                result.success = False
                result.error_message = err
                try:
                    goal_handle.abort()
                except Exception:
                    pass  # already in terminal state (client cancelled concurrently)
                return result

            # Acquire TEM and robot model once before the generator loop
            robot_model = self._moveit.get_robot_model()
            tem = self._moveit.get_trajectory_execution_manager()

            # Start look-ahead background planning thread (D-04)
            self._planner_service.plan_all(groups)

            # Generator loop — one iteration per planned group (D-05)
            for plan_dto in self._planner_service.iterate_planned_trajectories():

                # a. Cancel check (before executing anything in this group)
                if goal_handle.is_cancel_requested:
                    self._planner_service.cancel()  # idempotent; may already have been called
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.error_message = 'Goal was canceled'
                    result.trajectory_paths_completed = completed_ids  # D-02: partial list
                    try:
                        goal_handle.canceled()
                    except Exception:
                        pass
                    return result

                # b. Active-state check
                if not self._is_active:
                    self.get_logger().warn('Execution halted: node deactivated mid-trajectory')
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.error_message = 'Node deactivated during execution'
                    result.trajectory_paths_completed = completed_ids
                    try:
                        goal_handle.abort()
                    except Exception:
                        pass
                    return result

                # c. Planning failure check
                if not plan_dto.success:
                    self.get_logger().error(f'Look-ahead planning failed: {plan_dto.error_message}')
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.error_message = plan_dto.error_message
                    result.trajectory_paths_completed = completed_ids  # D-02: partial list
                    try:
                        goal_handle.abort()
                    except Exception:
                        pass
                    return result

                # d. Publish group-level 'executing' feedback (D-01)
                fb = ExecuteTrajectory.Feedback()
                fb.status = FeedbackStatusEnum.EXECUTING.value
                fb.trajectory_path_ids = plan_dto.path_ids
                goal_handle.publish_feedback(fb)

                # e. Get reference robot state for trajectory conversion
                with self._moveit.get_planning_scene_monitor().read_only() as scene:
                    ref_state = scene.current_state

                # f. Execute all segments in this group via TEM
                for ros_traj_msg in plan_dto.trajectories:
                    traj = RobotTrajectory(robot_model)
                    traj.set_robot_trajectory_msg(ref_state, ros_traj_msg)
                    tem.push(traj)
                try:
                    tem.execute_and_wait()
                except Exception as exec_err:
                    err_msg = f'TEM execution failed for paths {plan_dto.path_ids}: {exec_err}'
                    self.get_logger().error(err_msg)
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.error_message = err_msg
                    result.trajectory_paths_completed = completed_ids
                    try:
                        goal_handle.abort()
                    except Exception:
                        pass
                    return result

                # g. Publish group-level 'completed' feedback (D-01)
                fb2 = ExecuteTrajectory.Feedback()
                fb2.status = FeedbackStatusEnum.COMPLETED.value
                fb2.trajectory_path_ids = plan_dto.path_ids
                goal_handle.publish_feedback(fb2)

                # h. Track completed IDs (D-02)
                completed_ids.extend(plan_dto.path_ids)

            result = ExecuteTrajectory.Result()
            result.success = True
            result.error_message = ''
            result.trajectory_paths_completed = completed_ids
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = str(e)
            try:
                goal_handle.abort()
            except Exception:
                pass  # already in terminal state
            return result

        finally:
            with self._executing_lock:
                self._is_executing = False

    def _cancel_callback(self, cancel_request) -> CancelResponse:
        """Non-blocking cancel: signal planner to stop and return ACCEPT immediately (D-10).

        Does NOT join the planning thread. Does NOT call moveit.stop().
        The background thread terminates when it next checks cancel_event.
        The iterate_planned_trajectories() generator terminates via the StopIteration sentinel.
        """
        if self._planner_service is not None:
            self._planner_service.cancel()
        return CancelResponse.ACCEPT

    # endregion: action server callbacks


def main(args=None) -> None:
    rclpy.init(args=args)
    node = URMovementController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
