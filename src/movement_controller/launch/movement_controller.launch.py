# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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

#
# Author: Ron Freimann

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, EventHandler
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


ROBOT_TYPES = {
    "ur": ["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30"]
}


def declare_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur10_robot",
            description="Name of the robot being used",
        ),
        DeclareLaunchArgument(
            "model",
            description="Robot model being used.",
            choices=[
                models for models_list in ROBOT_TYPES.values() for models in models_list
            ],
            default_value="ur10",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.9",
            description="IP address of the robot controller",
        ),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Using or not time from simulation",
        ),
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
        
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Launch in debug mode with verbose logging",
        )
    ]


def get_robot_family(model_value: str) -> str:
    """Return the ROBOT_TYPES family key that the given model belongs to.

    Raises ValueError if the model is not part of any known family.
    """
    for family, models in ROBOT_TYPES.items():
        if model_value in models:
            return family
    raise ValueError(
        f"Model '{model_value}' does not map to any known robot family in ROBOT_TYPES."
    )


def build_moveit_config(family: str, model_value: str):
    """Build the MoveIt configuration for the given robot family."""
    if family == "ur":
        return (
            MoveItConfigsBuilder(robot_name=f"{model_value}_robot", package_name="movement_controller")
            .robot_description_semantic(
                str(Path("srdf") / "ur.srdf.xacro"), {"name": model_value}
            )
            .pilz_cartesian_limits()
            .planning_pipelines(
                default_planning_pipeline="pilz_industrial_motion_planner",
                pipelines=["pilz_industrial_motion_planner"],
            )
            .to_moveit_configs()
        )

    # Extend here with additional robot families, e.g.:
    # if family == "fanuc":
    #     return (
    #         MoveItConfigsBuilder(robot_name="fanuc", package_name="movement_controller")
    #         .robot_description_semantic(str(Path("srdf") / "fanuc.srdf.xacro"), {"name": model_value})
    #         .to_moveit_configs()
    #     )

    raise ValueError(f"No MoveIt configuration defined for robot family '{family}'.")

def load_speed_and_acceleration_constraints(family: str) -> dict:
    """Load the speed and acceleration limits for both joints and 
    cartesian motion from config file.
    """
    joint_limits_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "joint_limits.yaml"
    )
    if not joint_limits_file.is_file():
        raise FileNotFoundError(
            f"Speed and acceleration constraints file not found: {joint_limits_file}"
        )

    with joint_limits_file.open("r") as f:
        joint_data = yaml.safe_load(f) or {}

    cartesian_limits_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "pilz_cartesian_limits.yaml"
    )
    if not cartesian_limits_file.is_file():
        raise FileNotFoundError(
            f"Speed and acceleration constraints file not found: {cartesian_limits_file}"
        )

    with cartesian_limits_file.open("r") as f:
        cartesian_data = yaml.safe_load(f) or {}

    # find the strictest joint velocity and acceleration limits across all joints
    if family == "ur":
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        lowest_velocity_limit = min(
            [
                joint["max_velocity"] 
                for name, joint in joint_data["joint_limits"].items()
                if (name in joint_names and joint.get("has_velocity_limits", False))
            ]
        )
        lowest_acceleration_limit = min(
            [
                joint["max_acceleration"]
                for name, joint in joint_data["joint_limits"].items()
                if (name in joint_names and joint.get("has_acceleration_limits", False))
            ]
        )
    else:
        raise ValueError(f"No joint limits defined for robot family '{family}'.")
    
    # extract cartesian velocity and acceleration limits
    if "cartesian_limits" not in cartesian_data:
        raise KeyError(
            f"Cartesian limits file {cartesian_limits_file} has no 'cartesian_limits'."
        )
    cartesian_limits = cartesian_data["cartesian_limits"]
    max_cartesian_speed = cartesian_limits.get("max_trans_vel", 1.0)
    max_cartesian_acceleration = cartesian_limits.get("max_trans_acc", 2.25)

    return {
        "constraints.max_cartesian_speed": max_cartesian_speed,
        "constraints.max_cartesian_acceleration": max_cartesian_acceleration,
        "constraints.max_joint_speed": lowest_velocity_limit,
        "constraints.max_joint_acceleration": lowest_acceleration_limit,
    }

def load_joint_constraints(family: str) -> dict:
    """Load the joint constraints for the given robot family from config file.

    The file is expected at
    ``<share>/config/joint_constraints.yaml`` and to be
    keyed by the family name, e.g.::

        ur:
          shoulder_pan_joint:
            lower_limits: 2.0
            upper_limits: 4.5
          ...

    Returns the flattened parameter dict expected by the movement_controller
    node (parallel ``names``/``lower_limits``/``upper_limits`` lists).
    """
    constraints_file = (
        Path(get_package_share_directory("movement_controller"))
        / "config"
        / "joint_constraints.yaml"
    )
    if not constraints_file.is_file():
        raise FileNotFoundError(
            f"Joint constraints file not found for family '{family}': {constraints_file}"
        )

    with constraints_file.open("r") as f:
        data = yaml.safe_load(f) or {}

    if family not in data:
        raise KeyError(
            f"Joint constraints file {constraints_file} has no top-level key '{family}'."
        )

    joints = data[family]
    names = list(joints.keys())
    lower_limits = [float(joints[name]["lower_limits"]) for name in names]
    upper_limits = [float(joints[name]["upper_limits"]) for name in names]

    return {
        "constraints.joint.names": names,
        "constraints.joint.lower_limits": lower_limits,
        "constraints.joint.upper_limits": upper_limits,
    }


def setup_robot_nodes(context, *args, **kwargs):
    """Build the family-specific MoveIt config and the nodes that consume it.

    'model' is only known at runtime, so it is resolved here to select the
    matching robot family and its MoveIt configuration.
    """
    model_value = LaunchConfiguration("model").perform(context)
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
    debug = LaunchConfiguration("debug")

    family = get_robot_family(model_value)
    moveit_config = build_moveit_config(family, model_value)
    speed_and_acceleration_constraints = load_speed_and_acceleration_constraints(family)
    joint_constraints = load_joint_constraints(family)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("movement_controller"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(rviz),
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    movement_controller = Node(
        package="movement_controller",
        executable="movement_controller",
        name="movement_controller",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            IfElseSubstitution(
                condition=debug,
                if_value="DEBUG",
                else_value="INFO"
            )
        ],
        parameters=[
            joint_constraints,
            speed_and_acceleration_constraints
        ],
    )

    # start MoveGroup, RViz and the movement controller only once the robot
    # launcher signals (via the custom event) that it is ready
    return [
        RegisterEventHandler(
            EventHandler(
                matcher=lambda event: event.name == "robot_launched",
                entities=[move_group_node, rviz_node, movement_controller],
            )
        )
    ]


def generate_launch_description():
    # declare launch configurations
    model = LaunchConfiguration("model")
    robot_ip = LaunchConfiguration("robot_ip")

    # create launch description with declared launch arguments
    ld = LaunchDescription(declare_arguments())

    # include the correct robot driver launch file based on the selected robot model
    ur_control_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("movement_controller"), "launch", "ur.launch.py"]
        ),
        launch_arguments={
            "ur_type": model,
            "robot_ip": robot_ip,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", model, "' in ", str(ROBOT_TYPES["ur"])])
        ),
    )
    ld.add_action(ur_control_launch)

    # MoveIt config and the nodes that consume it are family-specific and are
    # built at runtime once the 'model' value can be resolved.
    ld.add_action(OpaqueFunction(function=setup_robot_nodes))

    return ld