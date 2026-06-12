#!/usr/bin/env python3
import rclpy
import uuid
from queue import Queue
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath

q = Queue()

def goal_response_callback(future):
    try:
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected')
            return

        print('Goal accepted')
        
        _get_result_future = goal_handle.get_result_async()
        _get_result_future.add_done_callback(get_result_callback)
    except Exception as e:
        print(f'Exception in goal response callback: {e}')

def get_result_callback(future):
    try:
        result = future.result().result
        print('Result received')
        if result is None:
            print('Failed to get result from action server')
            return

        if result.success:
            print('Motion executed successfully')
            print(f'Trajectory paths executed: {result.trajectory_paths_completed}')
        else:
            print('Motion execution failed')
    except Exception as e:
        print(f'Exception in get result callback: {e}')
        rclpy.shutdown()

def feedback_callback(feedback_msg):
    try:
        feedback = feedback_msg.feedback
        print(f'Feedback with status: {feedback.status} for path IDs: {feedback.trajectory_path_ids}')
    except Exception as e:
        print(f'Exception in feedback callback: {e}')

def main():
    rclpy.init()
    controller = Node('sandbox')
    action_client = ActionClient(controller, ExecuteTrajectory, 'movement_controller/execute_trajectory')
    
    if not action_client.wait_for_server(timeout_sec=5.0):
        controller.get_logger().error('ExecuteTrajectory action server not available')
        return
    
    # Example usage
    poses = []
    for xy in [
        (0.71984, -0.10547),
        (0.75984, -0.10547),
        (0.75984, -0.15547),
        (0.71984, -0.15547),
        (0.71984, -0.10547)
    ]:
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = controller.get_clock().now().to_msg()
        # target.pose.position.x = 0.3
        # target.pose.position.y = 0.2
        # target.pose.position.z = i
        # target.pose.orientation.w = 1.0
        target.pose.position.x = xy[0]
        target.pose.position.y = xy[1]
        target.pose.position.z = 0.6
        target.pose.orientation.x = 0.35728
        target.pose.orientation.y = 0.56644
        target.pose.orientation.z = 0.44363
        target.pose.orientation.w = 0.59556
        poses.append(target)
    
    controller.get_logger().info(f'Calling action')

    try:
        request = ExecuteTrajectory.Goal()
        paths = []
        for i, target in enumerate(poses):
            path = TrajectoryPath()
            path.target_pose = target
            path.motion_type = "LIN"
            path.path_id = str(uuid.uuid4())
            path.tool_frame = "tool0"
            # if i < 4:
            #     path.blend_radius = 0.005
            paths.append(path)

        request.paths = paths

        future = action_client.send_goal_async(request, feedback_callback=feedback_callback)
        future.add_done_callback(goal_response_callback)
        rclpy.spin(controller)
        
    except Exception as e:
        controller.get_logger().error(f'Exception while executing motion: {e}')
    finally:
        action_client.destroy()
        controller.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()