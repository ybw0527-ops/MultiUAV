from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    manager = Path(get_package_share_directory('falcon_exploration_manager'))
    renderer = Path(get_package_share_directory('falcon_map_render'))
    map_file = LaunchConfiguration('map_params_file')
    camera_file = LaunchConfiguration('sensor_params_file')
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='complex_office',
                              choices=['complex_office','classical_office','duplex_office','octa_maze','darpa_tunnel','power_plant']),
        DeclareLaunchArgument('map_params_file', default_value=PathJoinSubstitution([
            str(manager / 'config' / 'map'), [LaunchConfiguration('map_name'), '.yaml']])),
        DeclareLaunchArgument('sensor_params_file', default_value=str(manager / 'config' / 'robot' / 'uav_model_simulator.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz2 and publish the observed occupancy map'),
        DeclareLaunchArgument('execution_model',default_value='kinematic',choices=['kinematic','so3']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(manager / 'launch' / 'exploration.launch.py')),
            launch_arguments={'simulate_odometry':'true', 'sensor_pose_type':'transform',
                              'map_params_file':map_file, 'sensor_params_file':camera_file,
                              'use_sim_time':LaunchConfiguration('use_sim_time'),
                              'publish_occupancy_grid':LaunchConfiguration('rviz'),
                              'execution_model':LaunchConfiguration('execution_model')}.items()),
        Node(package='falcon_map_render', executable='map_render_node', output='screen',
             parameters=[str(renderer / 'config' / 'render.yaml'), camera_file, map_file,
                         {'use_sim_time':LaunchConfiguration('use_sim_time')}]),
        Node(package='rviz2', executable='rviz2', name='falcon_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(manager / 'config' / 'rviz2.rviz')],
             parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]),
    ])
