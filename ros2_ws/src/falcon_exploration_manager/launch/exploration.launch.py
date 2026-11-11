from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    def config(package, filename):
        return str(Path(get_package_share_directory(package)) / 'config' / filename)

    params = [config(package, filename) for package, filename in [
        ('falcon_voxel_mapping', 'voxel_mapping.yaml'),
        ('falcon_pathfinding', 'astar.yaml'),
        ('falcon_fast_planner', 'fast_planner.yaml'),
        ('falcon_exploration_preprocessing', 'frontier_finder.yaml'),
        ('falcon_exploration_preprocessing', 'hierarchical_grid.yaml'),
        ('falcon_exploration_manager', 'exploration_manager.yaml'),
    ]]
    params.append({'use_sim_time': LaunchConfiguration('use_sim_time')})
    params.append(LaunchConfiguration('sensor_params_file'))
    params.append(LaunchConfiguration('map_params_file'))
    params.append({'transformer.sensor_pose_topic': LaunchConfiguration('sensor_pose_topic'),
                   'transformer.pose_topic_type': LaunchConfiguration('sensor_pose_type')})
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odometry_topic', default_value='/uav_simulator/odometry'),
        DeclareLaunchArgument('sensor_params_file', default_value=config('falcon_exploration_manager', 'robot/uav_model_simulator.yaml')),
        DeclareLaunchArgument('map_params_file', default_value=config('falcon_voxel_mapping', 'voxel_mapping.yaml')),
        DeclareLaunchArgument('sensor_pose_topic', default_value='/uav_simulator/sensor_pose'),
        DeclareLaunchArgument('sensor_pose_type', default_value='odometry'),
        DeclareLaunchArgument('depth_image_topic', default_value='/uav_simulator/depth_image'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/uav_simulator/pointcloud'),
        DeclareLaunchArgument('simulate_odometry', default_value='false'),
        Node(package='falcon_exploration_manager', executable='exploration_node',
             output='screen', parameters=params,
             remappings=[('/odom_world', LaunchConfiguration('odometry_topic')),
                         ('/voxel_mapping/depth_image', LaunchConfiguration('depth_image_topic')),
                         ('/voxel_mapping/pointcloud', LaunchConfiguration('pointcloud_topic'))]),
        Node(package='falcon_fast_planner', executable='trajectory_server_node',
             output='screen', parameters=params),
        Node(package='falcon_poscmd_to_odom', executable='poscmd_to_odom_node',
             output='screen', condition=IfCondition(LaunchConfiguration('simulate_odometry')),
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('command', '/planning/pos_cmd'),
                         ('odometry', LaunchConfiguration('odometry_topic'))]),
    ])
