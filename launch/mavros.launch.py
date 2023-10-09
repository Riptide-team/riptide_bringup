import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    config_directory = os.path.join(ament_index_python.packages.get_package_share_directory('riptide_bringup'), 'config')

    config_file = PathJoinSubstitution(
        [
            FindPackageShare("riptide_bringup"),
            "config",
            "mavros.yaml",
        ]
    )
    mavros_node = launch_ros.actions.Node(package='mavros',
                                             executable='mavros_node',
                                             output='both',
                                             parameters=[config_file])

    return launch.LaunchDescription([mavros_node,
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=mavros_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])