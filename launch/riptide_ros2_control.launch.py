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

    # Robot state publisher
    # ld.add_action(
    #     Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         parameters=[robot_description],
    #     )
    # )

    # Joint state publisher
    # ld.add_action(
    #     Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         name='joint_state_publisher',
    #     )
    # )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        namespace=prefix,
        # prefix=['xterm -e gdb -ex run --args '], # gdb -ex run --args / valgrind --leak-check=full
        # arguments=["--ros-args", "--log-level", "debug"],
        output="both",
        emulate_tty=True
    )
    ld.add_action(controller_manager_node)

    # Joint state broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    # Imu sensor broadcaster
    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["imu_sensor_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    #     )
    # )

    # Pressure Broadcaster
    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace=prefix,
    #         arguments=["pressure_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    #     )
    # )

    # Imu Broadcaster
    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace=prefix,
    #         arguments=["imu_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
    #     )
    # )

    # Tail Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=prefix,
            arguments=["tail_broadcaster", "--controller-manager", "/" + prefix + "/controller_manager", "--unload-on-kill"],
        )
    )

    ld.add_action(
        Node(
            package="riptide_echosounder",
            executable="riptide_echosounder_driver",
            namespace=prefix,
            output="both"
        )
    )

    return ld