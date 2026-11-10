"""Launch the planning/execution graph in an isolated ROS domain and verify wiring."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from falcon_interfaces.msg import PositionCommand


def main():
    # Override only for this test and its descendants; leave user processes alone.
    os.environ['ROS_DOMAIN_ID'] = '177'
    rclpy.init()
    for config in (Path(__file__).parents[1] / 'config').rglob('*.yaml'):
        check = rclpy.create_node('config_check',
                                  cli_args=['--ros-args', '--params-file', str(config)],
                                  automatically_declare_parameters_from_overrides=True)
        check.destroy_node()
    node = rclpy.create_node('exploration_launch_smoke')
    received = {'odom': 0, 'command': 0}

    def count(key):
        def callback(_):
            received[key] += 1
        return callback

    subscriptions = [
        node.create_subscription(Odometry, '/uav_simulator/odometry', count('odom'), 10),
        node.create_subscription(PositionCommand, '/planning/pos_cmd', count('command'), 10),
    ]
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'falcon_exploration_manager', 'exploration.launch.py',
            'simulate_odometry:=true',
            'map_params_file:=' + str(Path(__file__).with_name('smoke.yaml')),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    break
                names = node.get_node_names()
                wired = (node.count_subscribers('/planning/bspline') >= 1 and
                         node.count_subscribers('/planning/replan') >= 1 and
                         node.count_subscribers('/uav_simulator/odometry') >= 1 and
                         node.count_subscribers('/uav_simulator/depth_image') >= 1)
                if ('exploration_node' in names and 'poscmd_to_odom' in names and
                        received['odom'] > 10 and received['command'] > 10 and wired):
                    print('PASS: exploration + trajectory server + simulated odometry; '
                          f'messages={received}; Bspline/replan/depth/odom links discovered')
                    return
            log.seek(0)
            raise RuntimeError('Launch graph smoke failed:\n' + log.read())
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
