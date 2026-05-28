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

from moveit.planning import MoveItPy
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
            'action_server_name',
            'movement_controller/execute_trajectory',
            ParameterDescriptor(description='ROS2 action server name for ExecuteTrajectory action'),
        )
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

        action_server_name = self.get_parameter('action_server_name').value
        moveit_group_name = self.get_parameter('moveit_group_name').value
        timeout: float = self.get_parameter('moveit_connection_timeout').value

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
            planning_component = self._moveit.get_planning_component(moveit_group_name)
            self._planner_service = PilzPlannerService(self._moveit, planning_component)
        except Exception as e:
            self.get_logger().error(f'MoveItPy initialisation failed: {e}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'MoveItPy connected; planning group: {moveit_group_name}')

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

            for group in groups:
                for path in group:
                    # Publish executing feedback per-path (D-15)
                    fb = ExecuteTrajectory.Feedback()
                    fb.status = FeedbackStatusEnum.EXECUTING.value
                    fb.trajectory_path_ids = [path.path_id]
                    goal_handle.publish_feedback(fb)

                    # Plan via PILZ — fail-fast on planning failure (D-16)
                    plan_result = self._planner_service.plan(path)
                    if not plan_result.success:
                        self.get_logger().error(
                            f'Planning failed for path {path.path_id!r}: {plan_result.error_message}'
                        )
                        result = ExecuteTrajectory.Result()
                        result.success = False
                        result.error_message = plan_result.error_message
                        goal_handle.abort()
                        return result

                    # Execute trajectory — NO blocking kwarg (Research Pitfall 4)
                    exec_status = self._moveit.execute(plan_result.trajectory, controllers=[])
                    if not exec_status:
                        err = f'Execution failed for path {path.path_id!r}'
                        self.get_logger().error(err)
                        result = ExecuteTrajectory.Result()
                        result.success = False
                        result.error_message = err
                        goal_handle.abort()
                        return result

                    # Publish completed feedback per-path (D-15)
                    fb2 = ExecuteTrajectory.Feedback()
                    fb2.status = FeedbackStatusEnum.COMPLETED.value
                    fb2.trajectory_path_ids = [path.path_id]
                    goal_handle.publish_feedback(fb2)
                    completed_ids.append(path.path_id)

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
