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
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from movement_controller.action import ExecuteTrajectory
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper


class URMovementController(LifecycleNode):
    """LifecycleNode that exposes an ExecuteTrajectory action server for UR robots."""

    def __init__(self, node_name: str = 'ur_movement_controller') -> None:
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._is_active: bool = False
        self._is_executing: bool = False
        self._executing_lock: Lock = Lock()

    # region: lifecycle callbacks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')

        self.declare_parameter(
            'action_server_name',
            'movement_controller/execute_trajectory',
            ParameterDescriptor(description='ROS2 action server name for ExecuteTrajectory action'),
        )
        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name (used from Phase 3 onward)'),
        )

        action_server_name = self.get_parameter('action_server_name').value
        self._action_server = ActionServer(
            self,
            ExecuteTrajectory,
            action_server_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f'Action server created at: {action_server_name}')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        self._is_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        self._is_active = False
        with self._executing_lock:
            self._is_executing = False  # signal any in-flight execution to wind down
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
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
        except ValidationError as e:
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

            for group in groups:
                path_ids = [p.path_id for p in group]

                fb = ExecuteTrajectory.Feedback()
                fb.status = FeedbackStatusEnum.EXECUTING.value
                fb.trajectory_path_ids = path_ids
                goal_handle.publish_feedback(fb)

                fb2 = ExecuteTrajectory.Feedback()
                fb2.status = FeedbackStatusEnum.COMPLETED.value
                fb2.trajectory_path_ids = path_ids
                goal_handle.publish_feedback(fb2)

            result = ExecuteTrajectory.Result()
            result.success = True
            result.error_message = ''
            result.trajectory_paths_completed = [p.path_id for p in goal_dto.paths]
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

        finally:
            with self._executing_lock:
                self._is_executing = False

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
