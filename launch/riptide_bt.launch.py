from launch import LaunchDescription
from launch_ros.actions.node import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


import os
import yaml
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()

    # Getting BT xml
    behavior_tree_xml = PathJoinSubstitution(
        [
            FindPackageShare("riptide_behavior_tree"),
            "bt",
            "one_second.xml"
        ]
    )

    # Getting YAML parameters
    behavior_tree_config_path = os.path.join(
        get_package_share_directory("riptide_bringup"),
        'config',
        'behavior_trees',
        'behavior_tree.yaml'
    )                       

    # Load the parameters specific to your ComposableNode
    with open(behavior_tree_config_path, 'r') as file:
        behavior_tree_config = yaml.safe_load(file)['riptide_behavior_tree']['ros__parameters']

    # Behavior tree node
    behavior_tree = Node(
        package='riptide_behavior_tree',
        executable='riptide_behavior_tree',
        parameters=[behavior_tree_config, {'bt_file_path': behavior_tree_xml}],
        output='both'
    )
    ld.add_action(behavior_tree)

    # Get URDF via xacro
    prefix = "Riptide_1"
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
        output="both",
    )
    # ld.add_action(controller_manager_node)

    return ld