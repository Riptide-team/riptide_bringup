from launch import LaunchDescription
from launch_ros.actions.node import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def load_broadcasters(ld):

    # Joint state broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--unload-on-kill"]
        )
    )

    # Imu sensor broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster", "--unload-on-kill"],
        )
    )

    # Pressure Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["pressure_broadcaster", "--unload-on-kill"],
        )
    )

    # Tail Broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["tail_broadcaster", "--unload-on-kill"],
        )
    )

    return ld


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
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        )
    )

    # Joint state publisher
    ld.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        )
    )

    # Echosounder_driver
    ld.add_action(
        Node(
            package="riptide_echosounder",
            executable="riptide_echosounder_driver",
            output="both"
        )
    )

    # Controller manager
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            # arguments=["--ros-args", "--log-level", "debug"],
            output="both",
            emulate_tty=True
        )
    )

    load_broadcasters(ld)

    # immersion_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["immersion_controller", "--inactive", "--unload-on-kill"],
        )
    )
    
    # riptide_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["riptide_controller", "--inactive", "--unload-on-kill"],
        )
    )

    # log_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["log_controller", "--inactive", "--unload-on-kill"],
        )
    )

    # depth_controller
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["depth_controller", "--inactive", "--unload-on-kill"],
        )
    )

    return ld