from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Get URDF via xacro
    prefix = "riptide"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("riptide_description"), "urdf", "riptide.urdf.xacro"]
            ),
            " ", "prefix:=", prefix
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("riptide_bringup"),
            "config",
            "test_actuators.yaml",
        ]
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
            namespace=prefix
        )
    )

    ld.add_action( 
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"publish_frequency": 100.}],
            namespace=prefix
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["test_actuators", "--controller-manager", "/controller_manager"],
            namespace=prefix
        )
    )

    return ld