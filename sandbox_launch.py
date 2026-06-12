from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.9",
        description="IP address of the robot controller",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz",
    )

    debug = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Launch in debug mode with verbose logging",
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "headless_mode": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur10",
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription([robot_ip_arg, launch_rviz_arg, debug, ur_control_launch, ur_moveit_launch])