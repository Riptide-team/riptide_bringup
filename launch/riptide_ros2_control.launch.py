from launch import LaunchDescription
from launch_ros.actions.node import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

from launch.actions import TimerAction


import os
import yaml
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()

    # Get URDF via xacro
    prefix = "riptide_1"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("riptide_description"), "urdf", "riptide.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("riptide_bringup"),
            "config",
            "controller_manager.yaml",
        ]
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        namespace=prefix,
        output="both",
    )
    ld.add_action(controller_manager_node)

    # Pressure Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["pressure_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    # Imu Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["imu_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    # Battery card Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["battery_card_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    # Actuators Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["tail_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    return ld