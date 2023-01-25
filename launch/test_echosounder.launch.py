from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro


def test_actuators(context: LaunchContext):
    namespace = LaunchConfiguration("namespace")

    # Getting xacro path
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("riptide_description"),
            "urdf",
            "riptide.urdf.xacro"
        ]
    ).perform(context)

    # Getting robot's urdf from xacro
    xacro_description = xacro.process_file(xacro_path, mappings={"prefix": context.perform_substitution(namespace)})

    robot_description = {"robot_description": xacro_description.toxml()}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("riptide_bringup"),
            "config",
            "test_echosounder.yaml",
        ]
    ).perform(context)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        namespace=context.perform_substitution(namespace)
    )

    controller_manager_topic = "/" + context.perform_substitution(namespace) + "/controller_manager"

    test_echosounder_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_echosounder", "--controller-manager", controller_manager_topic],
        namespace=context.perform_substitution(namespace)
    )

    return [controller_manager, test_echosounder_controller]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="riptide",
            description="Namespace used for the riptide",
        )
    )

    ld.add_action(
        OpaqueFunction(function=test_actuators)
    )

    return ld