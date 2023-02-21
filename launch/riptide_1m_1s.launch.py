from launch import LaunchDescription
from launch_ros.actions.node import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Get URDF via xacro
    prefix = "riptide_1"

    # Controller manager
    navigation_node = Node(
        package="riptide_navigation",
        executable="001-1s_1m.py",
        namespace=prefix,
        output="both",
    )
    ld.add_action(navigation_node)

    # Pressure Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["pressure_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    # Depth Controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["depth_controller", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    return ld