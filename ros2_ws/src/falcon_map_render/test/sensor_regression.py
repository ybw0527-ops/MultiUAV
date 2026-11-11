"""Render each real upstream map and fuse its depth through the ROS 2 MapServer."""
import array
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time
import sys
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    maps = ['complex_office','classical_office','duplex_office','octa_maze','darpa_tunnel','power_plant']
    selected = sys.argv[1:] or maps
    if any(name not in maps for name in selected):
        raise ValueError('Unknown map selection')
    if len(selected) > 1:
        # Fresh process and ROS domain per map avoid stale DDS endpoints between trials.
        for name in selected:
            subprocess.run([sys.executable, str(Path(__file__).resolve()), name], check=True)
        return
    os.environ['ROS_DOMAIN_ID'] = str(178 + maps.index(selected[0]))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    manager = Path(get_package_share_directory('falcon_exploration_manager'))
    voxel = Path(get_package_share_directory('falcon_voxel_mapping'))
    maps = selected
    try:
        for name in maps:
            config = manager / 'config' / 'map' / (name + '.yaml')
            params = yaml.safe_load(config.read_text())['/**']['ros__parameters']['map_config']
            node = rclpy.create_node('sensor_regression')
            stats = {'depth': 0, 'hits': 0, 'coverage': 0.0, 'paired': False}
            depth_stamps, pose_stamps = set(), set()

            def depth(msg):
                assert msg.encoding == '32FC1' and msg.header.frame_id == 'camera'
                assert msg.width == 80 and msg.height == 60
                values = array.array('f')
                values.frombytes(bytes(msg.data))
                if bool(msg.is_bigendian) != (os.sys.byteorder == 'big'):
                    values.byteswap()
                stats['depth'] += 1
                stats['hits'] = max(stats['hits'], sum(0.1 <= v <= 5.0 for v in values))
                depth_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))

            def pose(msg):
                assert msg.header.frame_id == 'world' and msg.child_frame_id == 'camera'
                pose_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))

            def coverage(msg):
                stats['coverage'] = max(stats['coverage'], msg.data)

            subscriptions = [
                node.create_subscription(Image, '/uav_simulator/depth_image', depth, qos_profile_sensor_data),
                node.create_subscription(TransformStamped, '/uav_simulator/sensor_pose', pose, 10),
                node.create_subscription(Float32, '/voxel_mapping/map_coverage', coverage, 10),
            ]
            odom_pub = node.create_publisher(Odometry, '/uav_simulator/odometry', 10)
            common = ['--ros-args', '--params-file', str(voxel/'config'/'voxel_mapping.yaml'),
                      '--params-file', str(manager/'config'/'robot'/'uav_model_simulator.yaml'),
                      '--params-file', str(config), '--params-file', str(Path(__file__).with_name('sensor_regression.yaml')),
                      '-r', '/voxel_mapping/depth_image:=/uav_simulator/depth_image']
            processes = []
            with tempfile.TemporaryFile(mode='w+') as log:
                try:
                    for package, executable in [('falcon_voxel_mapping','map_server_node'),('falcon_map_render','map_render_node')]:
                        binary = Path(get_package_prefix(package))/'lib'/package/executable
                        processes.append(subprocess.Popen([str(binary)] + common,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                    deadline = time.monotonic() + 45
                    next_publish = time.monotonic()
                    while time.monotonic() < deadline and all(p.poll() is None for p in processes):
                        if time.monotonic() >= next_publish:
                            odom = Odometry()
                            odom.header.stamp = node.get_clock().now().to_msg()
                            odom.header.frame_id = 'world'
                            odom.pose.pose.position.x = float(params.get('init_x',0))
                            odom.pose.pose.position.y = float(params.get('init_y',0))
                            odom.pose.pose.position.z = float(params.get('init_z',0)) + 1.0
                            odom.pose.pose.orientation.w = 1.0
                            odom_pub.publish(odom)
                            next_publish = time.monotonic() + 0.1
                        rclpy.spin_once(node, timeout_sec=0.05)
                        stats['paired'] = bool(depth_stamps & pose_stamps)
                        if stats['depth'] >= 3 and stats['hits'] > 0 and stats['coverage'] > 0 and stats['paired']:
                            print(f'PASS {name}: {stats}', flush=True)
                            break
                    else:
                        log.seek(0)
                        raise RuntimeError(f'{name}: {stats}; nodes={node.get_node_names()}; '
                                           f'odom readers={odom_pub.get_subscription_count()}; '
                                           f'depth publishers={node.count_publishers("/uav_simulator/depth_image")}; '
                                           f'exit codes={[p.poll() for p in processes]}\n' + log.read())
                finally:
                    for process in processes:
                        stop(process)
                    node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
