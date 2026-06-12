#!/usr/bin/env python3
import rclpy
import uuid
from queue import Queue
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, JointConstraint, Constraints, MotionSequenceItem, MotionSequenceRequest, PositionConstraint, OrientationConstraint, MotionPlanRequest, BoundingVolume
from moveit_msgs.srv import GetMotionSequence
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
]
lower_limits = [2.0, -2.0, -2.25, -4.71, -2.55]
upper_limits = [4.5, -0.2, -1.15, -2.78, -0.55]

def response_callback(future):
    try:
        result = future.result()
        if result is None:
            print('Service call failed')
            return
        if result.response.error_code.val != MoveItErrorCodes.SUCCESS:
            print(f'Service call failed with error code: {result.response.error_code.val} and message: {result.response.error_code.message}')
            return
        print(f'Service call succeeded')
    except Exception as e:
        print(f'Exception in response callback: {e}')

def get_goal_constraints(target: PoseStamped) -> list[Constraints]:
    position_constraint = PositionConstraint()
    position_constraint.weight = 1.0
    position_constraint.header.frame_id = target.header.frame_id
    position_constraint.link_name = "tool0"
    position_constraint.target_point_offset.x = 0.0
    position_constraint.target_point_offset.y = 0.0
    position_constraint.target_point_offset.z = 0.0
    speher = SolidPrimitive()
    speher.type = SolidPrimitive.SPHERE
    speher.dimensions = [0.01]
    speher_pose = Pose()
    speher_pose.position = target.pose.position
    speher_pose.orientation.w = 1.0
    bv = BoundingVolume()
    bv.primitives = [speher]
    bv.primitive_poses = [speher_pose]
    position_constraint.constraint_region = bv

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = target.header.frame_id
    orientation_constraint.link_name = "tool0"
    orientation_constraint.orientation = target.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 0.1
    orientation_constraint.weight = 1.0

    constraints = Constraints()
    constraints.name = f'goal_constraint'
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)

    print(f'Constructed goal constraints for link tool0:\n{constraints}')

    return [constraints]

def get_path_constraints(target: PoseStamped) -> Constraints:
    # For this example, we are not defining any path constraints
    constraints = Constraints()
    constraints.name = f'path_constraint'
    for name, lower, upper in zip(
        joint_names, lower_limits, upper_limits
    ):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = (lower + upper) / 2.0
        jc.tolerance_above = upper - jc.position
        jc.tolerance_below = jc.position - lower
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)  # type: ignore
    print(f'Constructed joint constraints:\n{constraints.joint_constraints}')
    return constraints

def main():
    rclpy.init()
    controller = Node('sandbox')
    planner_client = controller.create_client(GetMotionSequence, 'plan_sequence_path')
    
    if not planner_client.wait_for_service(timeout_sec=5.0):
        controller.get_logger().error('GetMotionSequence service server not available')
        return
    
    # Example usage
    poses = []
    for i in [0.58513]:
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.header.stamp = controller.get_clock().now().to_msg()
        # target.pose.position.x = 0.3
        # target.pose.position.y = 0.2
        # target.pose.position.z = i
        # target.pose.orientation.w = 1.0
        target.pose.position.x = 0.71984
        target.pose.position.y = -0.10547
        target.pose.position.z = i
        target.pose.orientation.x = 0.35728
        target.pose.orientation.y = 0.56644
        target.pose.orientation.z = 0.44363
        target.pose.orientation.w = 0.59556
        poses.append(target)
    
    controller.get_logger().info(f'Calling service')

    try:
        request = MotionSequenceRequest()
        items = []
        for target in poses:
            item = MotionSequenceItem()
            item.blend_radius = 0.0
            plan = MotionPlanRequest()
            plan.group_name = "ur_manipulator"
            plan.goal_constraints = get_goal_constraints(target)
            plan.path_constraints = get_path_constraints(target)
            plan.pipeline_id = 'pilz_industrial_motion_planner'
            plan.planner_id = "PTP"
            plan.allowed_planning_time = 5.0
            plan.num_planning_attempts = 10
            item.req = plan
            items.append(item)

        request.items = items
        print(f'===========================')
        print(f'Constructed request:\n{request}')

        service_request = GetMotionSequence.Request()
        service_request.request = request

        future = planner_client.call_async(service_request)
        print(f'Registering callback')
        future.add_done_callback(response_callback)
        print(f'Calling spin')
        rclpy.spin(controller)
        
    except Exception as e:
        controller.get_logger().error(f'Exception type: {type(e)}')
        controller.get_logger().error(f'Exception while executing motion: {str(e)}')
    finally:
        controller.destroy_client(planner_client)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()