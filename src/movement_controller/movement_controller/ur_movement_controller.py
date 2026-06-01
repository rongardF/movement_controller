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
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import CancelGoal
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.action import (
    ExecuteTrajectory_GetResult_Response as MoveitExecuteTrajectoryResponse,
    ExecuteTrajectory as MoveItExecuteTrajectory
)

from movement_controller.action import ExecuteTrajectory
from movement_controller.exceptions import (
    AbortPlanningError,
    ExecutionFailedError,
    NotInitializedError
)
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models import TrajectoryGoalDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper


class URMovementController(LifecycleNode):
    """LifecycleNode that exposes an ExecuteTrajectory action server for UR robots."""

    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._execute_trajectory_client: ActionClient | None = None
        self._is_executing: bool = False
        self._executing_lock: Lock = Lock()
        self._planner_service: PilzPlannerService | None = None
        self._trajectory_goal: TrajectoryGoalDTO | None = None

        # region: parameters
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
        # endregion: parameters

        # region: callback groups
        self._server_callback_group = ReentrantCallbackGroup()
        self._client_callback_group = ReentrantCallbackGroup()
        # endregion: callback groups

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')

        moveit_group_name = self.get_parameter('moveit_group_name').get_parameter_value().string_value
        self._planner_service = PilzPlannerService(node=self, moveit_group_name=moveit_group_name)
        self.get_logger().info('PilzPlannerService initialised successfully')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        try:
            if self._planner_service is not None:
                self._planner_service.on_activate()
            else:
                self.get_logger().error('PilzPlannerService not initialised during on_activate')
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'PilzPlannerService activated successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to activate PilzPlannerService: {e}')
            return TransitionCallbackReturn.FAILURE
        
        timeout = self.get_parameter('moveit_connection_timeout').get_parameter_value().double_value
        if self._planner_service is None or not self._planner_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'/plan_sequence_path service not available after {timeout}s — '
                'is move_group launched with pilz_industrial_motion_planner/MoveGroupSequenceService capability?'
            )
            return TransitionCallbackReturn.FAILURE
        
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            "movement_controller/execute_trajectory",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._server_callback_group,
        )
        self.get_logger().info(f'Action server created at: movement_controller/execute_trajectory')

        self._execute_trajectory_client = ActionClient(
            node=self,
            action_type=MoveItExecuteTrajectory, 
            action_name='/move_group/execute_trajectory',
            callback_group=self._client_callback_group
        )
        self.get_logger().info(f'Execute trajectory client created at: /move_group/execute_trajectory')

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if self._planner_service is not None:
            self._planner_service.cancel()  # ensure any ongoing planning is stopped if deactivated mid-plan
            self._planner_service.on_deactivate()
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
            self.get_logger().info('Action server destroyed successfully')
        if self._execute_trajectory_client is not None:
            self._execute_trajectory_client.destroy()
            self._execute_trajectory_client = None
            self.get_logger().info('Execute trajectory client destroyed successfully')
        with self._executing_lock:
            self._is_executing = False
        self._trajectory_goal = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        self._planner_service = None
        self._trajectory_goal = None
        return TransitionCallbackReturn.SUCCESS

    # endregion: lifecycle callbacks

    # region: private methods
    def _goal_abort(self, goal_handle: ServerGoalHandle) -> None:
        """Helper method to abort the current goal and reset executing state; 
        used in multiple places in _execute_callback to avoid code duplication.
        """
        try:
            goal_handle.abort()
        except Exception:
            self.get_logger().warning('Failed to abort goal handle - it may already be in a terminal state')
            pass
    
    # endregion: private methods

    # region: callbacks
    def _goal_callback(self, goal: ExecuteTrajectory.Goal) -> GoalResponse:
        # Only one goal at a time — check and set atomically to prevent TOCTOU race
        with self._executing_lock:
            if self._is_executing:
                self.get_logger().error('Goal rejected: another goal is already executing')
                return GoalResponse.REJECT
            self._is_executing = True

        # Full goal validation via DTO — reset executing flag on failure
        try:
            self._trajectory_goal = TrajectoryGoalDTO.from_ros_msg(goal)
        except (ValidationError, ValueError) as e:
            self.get_logger().error(f'Goal rejected: {e}')
            self._trajectory_goal = None
            with self._executing_lock:
                self._is_executing = False
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteTrajectory.Result:
        if self._trajectory_goal is None:
            goal = TrajectoryGoalDTO.from_ros_msg(goal_handle.request)
        else:
            goal = self._trajectory_goal  # use the already validated DTO from the goal callback
        groups = TrajectoryGrouper.group(goal.paths)
        completed_ids: list[str] = []
        error_message: str = ''

        if (
            self._planner_service is None or
            self._execute_trajectory_client is None or
            self._execute_trajectory_client.wait_for_server(timeout_sec=5.0) is False
        ):
            error_message = 'planner service or move_group client not initialised'
            self.get_logger().error(error_message)
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = error_message
            self._goal_abort(goal_handle)
            self._trajectory_goal = None  # reset after execution
            with self._executing_lock:
                self._is_executing = False
            return result

        try:
            if self._planner_service.plan_all(groups):  # start planning in background
                for plan in self._planner_service.iterate_planned_trajectories():
                    if (
                        goal_handle.is_cancel_requested or
                        not plan.success
                    ):
                        if goal_handle.is_cancel_requested:
                            error_message = "goal was canceled"
                        else:
                            error_message = f'look-ahead planning failed: {plan.error_message}'

                        self.get_logger().error(error_message)
                        result = ExecuteTrajectory.Result()
                        result.success = False
                        result.error_message = error_message
                        result.trajectory_paths_completed = completed_ids  # D-02: partial list
                        self._goal_abort(goal_handle)
                        self._trajectory_goal = None  # reset after execution
                        with self._executing_lock:
                            self._is_executing = False
                        return result
                    
                    fb = ExecuteTrajectory.Feedback()
                    fb.status = FeedbackStatusEnum.EXECUTING.value
                    fb.trajectory_path_ids = plan.path_ids
                    goal_handle.publish_feedback(fb)

                    for traj in plan.motion_plan.planned_trajectories:  # type: ignore[union-attr]
                        exec_goal = MoveItExecuteTrajectory.Goal()
                        exec_goal.trajectory = traj
                        execution_result: MoveitExecuteTrajectoryResponse = self._execute_trajectory_client.send_goal(exec_goal)  # type: ignore[assignment]
                        
                        if (
                            execution_result.status != 4 or  # SUCCEEDED
                            execution_result.result is None or
                            execution_result.result.error_code.val != MoveItErrorCodes.SUCCESS
                        ):
                            self._planner_service.cancel()  # ensure planning is stopped on exec failure
                            err_code = (
                                execution_result.result.error_code.val if execution_result.result else None
                            )
                            raise ExecutionFailedError(
                                f'failed to execute trajectory for paths {plan.path_ids}: '
                                f'error code {err_code}'
                            )
                    
                    fb2 = ExecuteTrajectory.Feedback()
                    fb2.status = FeedbackStatusEnum.COMPLETED.value
                    fb2.trajectory_path_ids = plan.path_ids
                    goal_handle.publish_feedback(fb2)

                    # h. Track completed IDs (D-02)
                    completed_ids.extend(plan.path_ids)

                result = ExecuteTrajectory.Result()
                result.success = True
                result.error_message = ''
                result.trajectory_paths_completed = completed_ids
                goal_handle.succeed()
                with self._executing_lock:
                    self._is_executing = False
                return result
        except AbortPlanningError as e:
            error_message = f'planning aborted: {e}'
        except ExecutionFailedError as e:
            error_message = f'execution failed: {e}'
        except NotInitializedError as e:
            error_message = f'not initialized: {e}'
        except Exception as e:
            error_message = f'unknown exception raised: {e}'

        self.get_logger().error(error_message)
        result = ExecuteTrajectory.Result()
        result.success = False
        result.error_message = error_message
        result.trajectory_paths_completed = completed_ids
        self._goal_abort(goal_handle)
        with self._executing_lock:
            self._is_executing = False
        self._trajectory_goal = None  # reset after execution
        return result

    def _cancel_callback(self, cancel_request: CancelGoal.Request) -> CancelResponse:
        """Non-blocking cancel: signal planner to stop and return ACCEPT immediately (D-10).

        Does NOT join the planning thread. Does NOT call moveit.stop().
        The background thread terminates when it next checks cancel_event.
        The iterate_planned_trajectories() generator terminates via the StopIteration sentinel.
        """
        if self._planner_service is not None:
            self._planner_service.cancel()
        return CancelResponse.ACCEPT

    # endregion: callbacks


def main(args=None) -> None:
    rclpy.init(args=args)
    node = URMovementController()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
