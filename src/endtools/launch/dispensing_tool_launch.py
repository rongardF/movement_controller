
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulated = LaunchConfiguration("simulated", default="true")
    tool_sn = LaunchConfiguration("tool_sn", default="true")
    touch_links = LaunchConfiguration("touch_links", default=["tool0", "tool_mount"])
    tcp_frame_id = LaunchConfiguration("tcp_frame_id", default="tool0")
    tcp = LaunchConfiguration("tcp", default="[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797]")
    mounted = LaunchConfiguration("mounted", default="false")

    declared_arguments = [
        
    ]

    endtool_node = Node(
        package="endtools",
        executable="dispensing_tool",
        output="screen",
        parameters=[
            {
                "simulated": simulated,
                "use_sim_time": simulated,
                "tool_sn": tool_sn,
                "touch_links": touch_links,
                "tcp_frame_id": tcp_frame_id,
                "tcp": tcp,
                "mounted": mounted,
                "collision_mesh": FindPackageShare("endtools") + "/config/meshes/dispensing_tool_collision.stl",
            },
        ],
    )

    return LaunchDescription(declared_arguments + [endtool_node])