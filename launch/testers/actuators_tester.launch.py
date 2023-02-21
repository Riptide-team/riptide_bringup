from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    prefix = "riptide_1"

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["actuator_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    return ld