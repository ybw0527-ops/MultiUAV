from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    default_parameters = os.path.join(
        get_package_share_directory("falcon_voxel_mapping"),
        "config",
        "voxel_mapping.yaml",
    )

    parameters_file = LaunchConfiguration("parameters_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="ROS 2 parameters for the FALCON voxel map server",
            ),
            Node(
                package="falcon_voxel_mapping",
                executable="map_server_node",
                name="falcon_map_server",
                output="screen",
                parameters=[parameters_file],
            ),
        ]
    )
