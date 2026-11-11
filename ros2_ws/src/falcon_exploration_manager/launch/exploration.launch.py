from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    params.append({'fast_planner.velocity_scale':ParameterValue(PythonExpression([
        "0.85 if '",LaunchConfiguration('execution_model'),"' == 'so3' else 1.0"]),value_type=float),
                   'fast_planner.acceleration_scale':ParameterValue(PythonExpression([
        "0.60 if '",LaunchConfiguration('execution_model'),"' == 'so3' else 1.0"]),value_type=float)})
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
        DeclareLaunchArgument('execution_model',default_value='kinematic',choices=['kinematic','so3']),
        Node(package='falcon_exploration_manager', executable='exploration_node',
             output='screen', parameters=params,
             remappings=[('/odom_world', LaunchConfiguration('odometry_topic')),
                         ('/voxel_mapping/depth_image', LaunchConfiguration('depth_image_topic')),
                         ('/voxel_mapping/pointcloud', LaunchConfiguration('pointcloud_topic'))]),
        Node(package='falcon_fast_planner', executable='trajectory_server_node',
             output='screen', parameters=params),
        GroupAction(condition=IfCondition(LaunchConfiguration('simulate_odometry')),actions=[
        Node(package='falcon_poscmd_to_odom', executable='poscmd_to_odom_node',
             output='screen', condition=LaunchConfigurationEquals('execution_model','kinematic'),
             parameters=[LaunchConfiguration('map_params_file'),{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('command', '/planning/pos_cmd'),
                         ('odometry', LaunchConfiguration('odometry_topic'))]),
        Node(package='falcon_so3_simulator',executable='so3_simulator_node',output='screen',
             condition=LaunchConfigurationEquals('execution_model','so3'),
             parameters=[config('falcon_so3_simulator','simulator.yaml'),
                         LaunchConfiguration('sensor_params_file'),LaunchConfiguration('map_params_file'),
                         {'use_sim_time':LaunchConfiguration('use_sim_time')}],
             remappings=[('command','/planning/pos_cmd'),('odometry',LaunchConfiguration('odometry_topic')),
                         ('imu','/uav_simulator/imu'),('motor_rpm','/uav_simulator/motor_rpm'),
                         ('so3_cmd','/uav_simulator/so3_cmd')]),
        ]),
    ])
