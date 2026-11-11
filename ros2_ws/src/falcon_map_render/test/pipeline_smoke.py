"""Bounded full-graph test: simulated odometry -> renderer -> manager/map fusion."""
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32


def main():
    os.environ['ROS_DOMAIN_ID'] = '179'
    rclpy.init()
    node = rclpy.create_node('render_pipeline_smoke')
    observed = {'depth': 0, 'coverage': 0.0, 'frontier_points': 0}

    def depth(_):
        observed['depth'] += 1

    def coverage(msg):
        observed['coverage'] = max(observed['coverage'], msg.data)

    def frontier(msg):
        observed['frontier_points'] = max(observed['frontier_points'], msg.width * msg.height)

    subscriptions = [
        node.create_subscription(Image, '/uav_simulator/depth_image', depth, qos_profile_sensor_data),
        node.create_subscription(Float32, '/voxel_mapping/map_coverage', coverage, 10),
        node.create_subscription(PointCloud2, '/planning_vis/frontier_pcl', frontier, 10),
        node.create_subscription(PointCloud2, '/planning_vis/dormant_frontier_pcl', frontier, 10),
        node.create_subscription(PointCloud2, '/planning_vis/tiny_frontier_pcl', frontier, 10),
    ]
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            'ros2','launch','falcon_map_render','simulation.launch.py',
            'sensor_params_file:=' + str(Path(__file__).with_name('sensor_regression.yaml')),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if observed['depth'] >= 5 and observed['coverage'] > 0 and observed['frontier_points'] > 0:
                    print('PASS full simulation graph:', observed, flush=True)
                    return
            log.seek(0)
            raise RuntimeError(f'Full simulation graph failed: {observed}\n' + log.read())
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
