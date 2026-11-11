"""Bounded live exploration: measure executed motion and mapping after first plan."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from falcon_interfaces.msg import Bspline, PositionCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32


def main():
    parser = argparse.ArgumentParser(__doc__)
    maps = ['darpa_tunnel', 'power_plant', 'complex_office', 'duplex_office', 'classical_office', 'octa_maze']
    parser.add_argument('--map', choices=maps + ['all'], default='complex_office')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', default='/tmp/falcon_autonomous')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 10:
        parser.error('--duration must be finite and at least 10 seconds')
    if args.map == 'all':
        summary = []
        for index, name in enumerate(maps):
            child = subprocess.run([sys.executable, __file__, '--map', name,
                                    '--duration', str(args.duration), '--output', args.output],
                                   env={**os.environ, 'ROS_DOMAIN_ID': str(190 + index)}, check=False)
            report = Path(args.output) / (name + '.json')
            metrics = json.loads(report.read_text()) if report.exists() else {}
            summary.append({**{k: v for k, v in metrics.items() if k != 'samples'},
                            'map': name, 'passed': child.returncode == 0})
        Path(args.output, 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        raise SystemExit(0 if all(row['passed'] for row in summary) else 1)
    os.environ.setdefault('ROS_DOMAIN_ID', '187')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = rclpy.create_node('autonomous_regression')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    start = time.monotonic()
    result = dict(map=args.map, trajectories=0, commands=0, invalid_commands=0,
                  coverage_start=None, coverage_final=0.0, distance=0.0,
                  command_jumps=0, max_command_step=0.0, max_speed=0.0, max_acceleration=0.0)
    result['requested_duration_sec'] = args.duration
    result['max_spline_speed_bound'] = 0.0
    result['max_spline_acceleration_bound'] = 0.0
    result['max_spline_yaw_rate_bound'] = 0.0
    result['max_spline_yaw_acceleration_bound'] = 0.0
    result['max_yaw_rate'] = 0.0
    result['max_command_yaw_acceleration'] = 0.0
    result['max_command_linear_acceleration'] = 0.0
    result['kinematic_position_inconsistencies'] = 0
    result['braking_command_samples'] = 0
    result['completed_stop_episodes'] = 0
    result['yaw_command_jumps'] = 0
    result['shutdown_forced'] = False
    result['frontier_points'] = {'active': 0, 'dormant': 0, 'tiny': 0}
    samples = []
    previous = None
    previous_command = None

    def trajectory(msg):
        result['trajectories'] += 1
        for yaw in (False, True):
            points = [(p,) for p in msg.yaw_pts] if yaw else [(p.x, p.y, p.z) for p in msg.pos_pts]
            knots = [msg.yaw_dt * (i-3) for i in range(len(points)+4)] if yaw else list(msg.knots)
            for derivative in (1, 2):
                degree = 4-derivative if yaw else msg.order-derivative+1
                next_points = []
                for i in range(len(points) - 1):
                    denominator = knots[i + degree + 1] - knots[i + 1]
                    if denominator <= 0:
                        next_points.append((math.inf,) * len(points[i]))
                    else:
                        next_points.append(tuple(degree * (b-a) / denominator for a, b in zip(points[i], points[i+1])))
                points, knots = next_points, knots[1:-1]
                keys = ('max_spline_yaw_rate_bound', 'max_spline_yaw_acceleration_bound') if yaw else ('max_spline_speed_bound', 'max_spline_acceleration_bound')
                key = keys[derivative-1]
                result[key] = max(result[key], max(math.hypot(*p) for p in points))
        if result['coverage_start'] is None:
            result['coverage_start'] = result['coverage_final']
            result['first_plan_sec'] = time.monotonic() - start

    def coverage(msg):
        result['coverage_final'] = float(msg.data)

    def command(msg):
        nonlocal previous_command
        result['commands'] += 1
        values = [msg.position.x, msg.position.y, msg.position.z,
                  msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.yaw, msg.yaw_dot,
                  msg.acceleration.x, msg.acceleration.y, msg.acceleration.z,
                  msg.jerk.x, msg.jerk.y, msg.jerk.z]
        if not all(math.isfinite(x) for x in values):
            result['invalid_commands'] += 1
        if result['trajectories']:
            point = (msg.position.x, msg.position.y, msg.position.z)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if previous_command is not None:
                step = math.dist(point, previous_command[1])
                result['max_command_step'] = max(result['max_command_step'], step)
                if step > 0.5 + 3.0 * max(0.0, stamp - previous_command[0]):
                    result['command_jumps'] += 1
                dt = stamp - previous_command[0]
                if 1e-3 < dt < 0.2:
                    yaw_step = abs(math.remainder(msg.yaw-previous_command[2],2*math.pi))
                    if yaw_step > 0.05 + math.pi/2*dt:
                        result['yaw_command_jumps'] += 1
                    result['max_command_yaw_acceleration'] = max(result['max_command_yaw_acceleration'], abs(msg.yaw_dot-previous_command[3])/dt)
                    velocity=tuple(values[3:6])
                    linear_acceleration=math.dist(velocity,previous_command[4])/dt
                    result['max_command_linear_acceleration']=max(result['max_command_linear_acceleration'],linear_acceleration)
                    position_error=math.hypot(*(b-a-0.5*dt*(u+v) for a,b,u,v in zip(previous_command[1],point,previous_command[4],velocity)))
                    if position_error > dt*dt+0.001:
                        result['kinematic_position_inconsistencies'] += 1
                if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_COMPLETED and previous_command[5] != msg.trajectory_flag:
                    result['completed_stop_episodes'] += 1
            if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_ABORT:
                result['braking_command_samples'] += 1
            previous_command = (stamp, point, msg.yaw, msg.yaw_dot, tuple(values[3:6]), msg.trajectory_flag)
            result['max_speed'] = max(result['max_speed'], math.hypot(*values[3:6]))
            result['max_acceleration'] = max(result['max_acceleration'], math.hypot(*values[8:11]))
            result['max_yaw_rate'] = max(result['max_yaw_rate'], abs(msg.yaw_dot))

    def odom(msg):
        nonlocal previous
        p = msg.pose.pose.position
        point = (p.x, p.y, p.z)
        if result['trajectories']:
            if previous is not None:
                step = math.dist(point, previous)
                result['distance'] += step
                if step > 1e-4:
                    result['last_motion_sec'] = time.monotonic() - start
            samples.append([time.monotonic() - start, *point])
            previous = point

    subscriptions = [
        node.create_subscription(Bspline, '/planning/bspline', trajectory, 10),
        node.create_subscription(PositionCommand, '/planning/pos_cmd', command, 10),
        node.create_subscription(Odometry, '/uav_simulator/odometry', odom, qos_profile_sensor_data),
        node.create_subscription(Float32, '/voxel_mapping/map_coverage', coverage, 10),
    ]
    for category, topic in [('active','frontier'), ('dormant','dormant_frontier'), ('tiny','tiny_frontier')]:
        subscriptions.append(node.create_subscription(PointCloud2, '/planning_vis/' + topic + '_pcl',
            lambda msg, name=category: result['frontier_points'].__setitem__(name, msg.width * msg.height), 10))
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / (args.map + '.log')
    sensor_config = Path(__file__).with_suffix('.yaml')
    if args.map == 'power_plant':
        config = yaml.safe_load(sensor_config.read_text())
        mapping = config['/**']['ros__parameters']['voxel_mapping']
        mapping.update(resolution_fine=0.1, resolution_coarse=0.2, skip_pixel=1)
        # Match CPU fusion throughput; queuing 10 dense fine-map frames per
        # second starves the mutually exclusive planner callbacks.
        config['/**']['ros__parameters']['map_render']['render_rate'] = 3.0
        sensor_config = output / 'power_plant_parameters.yaml'
        sensor_config.write_text(yaml.safe_dump(config))
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'falcon_map_render', 'simulation.launch.py',
            'map_name:=' + args.map,
            'use_sim_time:=true',
            'sensor_params_file:=' + str(sensor_config.resolve()),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            heartbeat = start + 10
            while time.monotonic() - start < args.duration and process.poll() is None:
                clock = Clock()
                elapsed_ns = int((time.monotonic() - start + 1.0) * 1e9)
                clock.clock.sec, clock.clock.nanosec = divmod(elapsed_ns, 10**9)
                clock_pub.publish(clock)
                rclpy.spin_once(node, timeout_sec=0.1)
                diagnostic = log_path.read_text()
                if any(error in diagnostic for error in
                       ['process has died', '[FATAL]', 'process has finished', 'Rejected B-spline']):
                    raise RuntimeError('Simulation node failed; see ' + str(log.name))
                if time.monotonic() >= heartbeat:
                    print(json.dumps(result), flush=True)
                    heartbeat += 10
            if time.monotonic() - start < args.duration - 0.2:
                raise RuntimeError('Launch exited before the requested regression duration')
            result['fsm_finished'] = '[FSM] Exploration finished.' in diagnostic
            result['elapsed_sec'] = time.monotonic() - start
        finally:
            if process.poll() is None:
                # Let launch signal its own nodes once. Signaling the entire
                # process group as well delivered duplicate SIGINTs to nodes.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    result['shutdown_forced'] = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            result['samples'] = samples
            (output / (args.map + '.json')).write_text(json.dumps(result, indent=2))
    teardown_log = log_path.read_text()
    if (result['shutdown_forced'] or teardown_log.count('process has finished cleanly') != 4 or
            'process has died' in teardown_log or '[FATAL]' in teardown_log):
        raise RuntimeError('Simulation node failed during shutdown; see ' + str(log_path))
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}), flush=True)
    assert result['invalid_commands'] == 0, 'Nonfinite flight command'
    assert result['max_speed'] <= 2.0 + 1e-5, 'Command speed exceeds 2 m/s'
    assert result['max_acceleration'] <= 2.0 + 1e-5, 'Command acceleration exceeds 2 m/s^2'
    assert result['max_spline_speed_bound'] <= 2.0 + 1e-5, 'Whole spline exceeds speed limit'
    assert result['max_spline_acceleration_bound'] <= 2.0 + 1e-5, 'Whole spline exceeds acceleration limit'
    assert result['max_yaw_rate'] <= math.pi/2 + 1e-5, 'Command yaw rate exceeds limit'
    assert result['max_spline_yaw_rate_bound'] <= math.pi/2 + 1e-5, 'Whole yaw spline exceeds rate limit'
    assert result['max_spline_yaw_acceleration_bound'] <= math.pi/2 + 1e-5, 'Whole yaw spline exceeds acceleration limit'
    assert result['yaw_command_jumps'] == 0, 'Flight command yaw jumped'
    assert result['max_command_yaw_acceleration'] <= math.pi/2 + 0.05, 'Command yaw rate jumped'
    assert result['max_command_linear_acceleration'] <= 2.0 + 1e-3, 'Command linear velocity jumped'
    assert result['kinematic_position_inconsistencies'] == 0, 'Command position disagrees with velocity'
    assert result['command_jumps'] == 0, 'Flight command position jumped'
    assert result['trajectories'] >= 2, 'No exploration/replanning trajectory'
    assert result['distance'] > 1.0, 'No executed exploration motion'
    assert result['fsm_finished'] or args.duration - result.get('last_motion_sec', 0) < 15.0, 'Exploration stalled before timeout'
    assert result['coverage_final'] > result['coverage_start'] + 1.0, 'No coverage growth after planning'
    config = yaml.safe_load((Path(get_package_share_directory('falcon_exploration_manager')) /
                             'config' / 'map' / (args.map + '.yaml')).read_text())['/**']['ros__parameters']['map_config']
    bounds = config['map_size']
    result['task_box_volume'] = math.prod(bounds['box_max_' + axis] - bounds['box_min_' + axis] for axis in 'xyz')
    result['known_task_box_fraction'] = result['coverage_final'] / result['task_box_volume']
    xyz = output / (args.map + '.xyz')
    xyz.write_text(''.join(' '.join(str(v) for v in sample[1:]) + '\n' for sample in samples))
    scene = Path(config['map_file'])
    if not scene.is_absolute():
        scene = Path(get_package_share_directory('falcon_map_render')) / 'resource' / scene
    oracle = subprocess.run([
        str(Path(get_package_prefix('falcon_map_render')) / 'lib/falcon_map_render/check_clearance'),
        str(scene), str(config.get('scale', 1.0)),
        *[str(v) for v in config.get('T_m_w', [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])], str(xyz), '0.2',
    ], text=True, capture_output=True, check=False)
    if oracle.returncode not in (0, 2):
        raise RuntimeError(oracle.stderr)
    result['ground_truth'] = json.loads(oracle.stdout)
    (output / (args.map + '.json')).write_text(json.dumps(result, indent=2))
    print(oracle.stdout.strip(), flush=True)
    if oracle.returncode:
        print(oracle.stderr.strip(), flush=True)
    assert oracle.returncode == 0, 'Insufficient ground-truth clearance'


if __name__ == '__main__':
    main()
