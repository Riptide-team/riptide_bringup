from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="riptide_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
                Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="riptide_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_package",
            default_value="riptide_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
                is not set, it enables use of a custom description.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_file",
            description="URDF/XACRO description file with the robot.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
                multi-robot setup. If changed than also joint names in the controllers' configuration \
                have to be updated.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Riptide controller to start.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "record_rosbag",
            default_value="true",
            description="Record a rosbag automatically with this launch file.",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    record_rosbag = LaunchConfiguration("record_rosbag")


    # Get URDF via xacro
    prefix = "Riptide_1"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controllers_file,
        ]
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
        )
    )

    ld.add_action( 
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"publish_frequency": 1000.}],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["state_estimator", "--controller-manager", "/controller_manager"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["echosounder_controller", "--controller-manager", "/controller_manager"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["riptide_controller", "--controller-manager", "/controller_manager"],
        )
    )

    # Delay joint_state_broadcaster after robot_state_publisher
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_state_publisher,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    # orthogonal_controller
    ld.add_action(
        Node(
            package="riptide_navigation",
            executable="orthogonal_controller",
            output="both"
        )
    )

    # state_machine
    ld.add_action(
        Node(
            package="riptide_navigation",
            executable="state_machine",
            output="both"
        )
    )

    # rosbag record
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
            condition=IfCondition(record_rosbag)
        )
    )

    # orthogonal_controller
    stable_cycles = Node(
        package="riptide_navigation",
        executable="stable_cycles",
        output="both"
    )

    return ld