from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    planner=Path(get_package_share_directory('falcon_fast_planner'))
    model=Path(get_package_share_directory('falcon_so3_simulator'))
    pose={'map_config.init_x':0.0,'map_config.init_y':0.0,'map_config.init_z':2.0,
          'use_sim_time':True,'fast_planner.velocity_scale':0.85,'fast_planner.acceleration_scale':0.60}
    return LaunchDescription([
        DeclareLaunchArgument('model_clock_topic',default_value='/clock'),
        Node(package='falcon_fast_planner',executable='trajectory_server_node',output='screen',
             parameters=[str(planner/'config/fast_planner.yaml'),pose]),
        Node(package='falcon_so3_simulator',executable='so3_simulator_node',output='screen',
             parameters=[str(model/'config/simulator.yaml'),pose],
             remappings=[('/clock',LaunchConfiguration('model_clock_topic')),
                         ('command','/planning/pos_cmd'),('odometry','/uav_simulator/odometry'),
                         ('motor_rpm','/uav_simulator/motor_rpm'),
                         ('state_trace','/uav_simulator/state_trace'),
                         ('integration_status','/uav_simulator/integration_status')]),
    ])
