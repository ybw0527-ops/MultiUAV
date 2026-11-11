"""Live initialization, clock pause, and mid-maneuver braking through SO3 physics."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ.setdefault('ROS_DOMAIN_ID','187')
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from falcon_interfaces.msg import PositionCommand
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Int32, Float64MultiArray

def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--stop',choices=['none','vertical','horizontal'],default='none')
    parser.add_argument('--output',default='/tmp/falcon_initialization_tracking')
    parser.add_argument('--duration',type=float,default=9.0)
    args=parser.parse_args()
    rclpy.init(); node=rclpy.create_node('initialization_tracking_regression')
    clock_pub=node.create_publisher(Clock,'/clock',10)
    stop_pub=node.create_publisher(Int32,'/planning/replan',10)
    result=dict(scenario=args.stop,commands=0,odometry_samples=0,invalid=0,ready=False,
                max_command_speed=0.0,max_command_acceleration=0.0,max_command_fd_acceleration=0.0,
                max_actual_speed=0.0,max_actual_acceleration=0.0,max_tracking_error=0.0,
                motor_samples=0,max_motor_rpm=0.0,stop_requested=False,stopped=False,
                clock_pause_samples=0,clock_pause_position_drift=0.0,forced_shutdown=False)
    result['ros_domain_id']=os.environ['ROS_DOMAIN_ID']
    result['rmw_implementation']=os.environ.get('RMW_IMPLEMENTATION','default')
    last_command=None; previous=None; actual=None; previous_actual=None; pause_anchor=None
    pause_time=None; pause_started=None; paused_duration=0.0; stop_reference=None
    motion_start=None
    def command(msg):
        nonlocal last_command, previous, pause_anchor, motion_start
        if motion_start is None: motion_start=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        point=(msg.position.x,msg.position.y,msg.position.z)
        velocity=(msg.velocity.x,msg.velocity.y,msg.velocity.z)
        acceleration=(msg.acceleration.x,msg.acceleration.y,msg.acceleration.z)
        if not all(math.isfinite(x) for x in (*point,*velocity,*acceleration)): result['invalid']+=1
        result['commands']+=1
        result['max_command_speed']=max(result['max_command_speed'],math.hypot(*velocity))
        result['max_command_acceleration']=max(result['max_command_acceleration'],math.hypot(*acceleration))
        if previous and 1e-3<stamp-previous[0]<0.2:
            result['max_command_fd_acceleration']=max(result['max_command_fd_acceleration'],math.dist(velocity,previous[2])/(stamp-previous[0]))
        previous=last_command=(stamp,point,velocity,msg.trajectory_flag)
        if msg.trajectory_flag==PositionCommand.TRAJECTORY_STATUS_COMPLETED: result['stopped']=True
        if pause_time is not None and abs(stamp-pause_time)<1e-6:
            if pause_anchor is None: pause_anchor=point
            result['clock_pause_samples']+=1
            result['clock_pause_position_drift']=max(result['clock_pause_position_drift'],math.dist(point,pause_anchor))
    def odom(msg):
        nonlocal actual,previous_actual
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        point=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
        velocity=(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z)
        if not all(math.isfinite(x) for x in (*point,*velocity)): result['invalid']+=1
        result['odometry_samples']+=1
        result['max_actual_speed']=max(result['max_actual_speed'],math.hypot(*velocity))
        if previous_actual and 1e-3<stamp-previous_actual[0]<0.2:
            result['max_actual_acceleration']=max(result['max_actual_acceleration'],math.dist(velocity,previous_actual[2])/(stamp-previous_actual[0]))
        if last_command and abs(stamp-last_command[0])<0.05:
            result['max_tracking_error']=max(result['max_tracking_error'],math.dist(point,last_command[1]))
        previous_actual=actual=(stamp,point,velocity)
    def motors(msg):
        if len(msg.data)!=4 or not all(math.isfinite(x) and 0<=x<=35000 for x in msg.data): result['invalid']+=1
        result['motor_samples']+=1
        result['max_motor_rpm']=max(result['max_motor_rpm'],max(msg.data))
    subs=[node.create_subscription(PositionCommand,'/planning/pos_cmd',command,20),
          node.create_subscription(Odometry,'/uav_simulator/odometry',odom,qos_profile_sensor_data),
          node.create_subscription(Float64MultiArray,'/uav_simulator/motor_rpm',motors,20),
          node.create_subscription(Bool,'/planning/initialization_complete',lambda m:result.__setitem__('ready',m.data),
                                   QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    path=output/(args.stop+'.log'); start=time.monotonic(); process=None
    with path.open('w') as log:
        process=subprocess.Popen(['ros2','launch',str(Path(__file__).with_name('initialization_tracking.launch.py'))],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            heartbeat=start+5
            while motion_start is None or time.monotonic()-motion_start<args.duration:
                wall=time.monotonic()
                if motion_start is None and wall-start>30: raise RuntimeError('Initialization node startup timed out')
                if args.stop=='none' and last_command and pause_started is None and math.hypot(*last_command[2])>0.3:
                    pause_started=wall; pause_time=wall-motion_start+1.0
                if pause_started is not None and wall-pause_started<0.25:
                    seconds=pause_time
                else:
                    paused_duration=0.25 if pause_started is not None else 0.0
                    seconds=1.0 if motion_start is None else wall-motion_start+1.0-paused_duration
                msg=Clock(); msg.clock.sec,msg.clock.nanosec=divmod(round(seconds*1e9),10**9); clock_pub.publish(msg)
                rclpy.spin_once(node,timeout_sec=0.002)
                if wall>=heartbeat:
                    print(json.dumps(dict(nodes=node.get_node_names(),clock_subscribers=clock_pub.get_subscription_count(),
                        command_publishers=len(node.get_publishers_info_by_topic('/planning/pos_cmd')),
                        clock=seconds,commands=result['commands'])),flush=True)
                    heartbeat=wall+5
                if last_command and args.stop!='none' and not result['stop_requested']:
                    axis=2 if args.stop=='vertical' else 0
                    if last_command[2][axis]>0.4 and (axis==2 or last_command[1][0]>0.2):
                        stop=Int32(); stop.data=1; stop_pub.publish(stop)
                        result['stop_requested']=True; stop_reference=last_command
                if process.poll() is not None: raise RuntimeError('Node exited early; see '+str(path))
        finally:
            result['discovered_nodes']=node.get_node_names()
            result['clock_subscribers']=clock_pub.get_subscription_count()
            result['command_publishers']=len(node.get_publishers_info_by_topic('/planning/pos_cmd'))
            print(json.dumps(dict(connection_diagnostic=result)),flush=True)
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                result['forced_shutdown']=True; os.killpg(process.pid,signal.SIGKILL); process.wait()
            node.destroy_node(); rclpy.shutdown()
    if actual and last_command:
        result['final_tracking_error']=math.dist(actual[1],last_command[1]); result['final_actual_speed']=math.hypot(*actual[2])
        result['final_command_position']=last_command[1]
    if stop_reference:
        result['reference_stop_distance']=math.dist(stop_reference[1],last_command[1])
        result['ideal_stop_distance_at_request']=math.hypot(*stop_reference[2])**2/(2*1.2)
    text=path.read_text()
    result['startup_wait_sec']=None if motion_start is None else motion_start-start
    (output/(args.stop+'.json')).write_text(json.dumps(result,indent=2)); print(json.dumps(result),flush=True)
    assert not result['forced_shutdown'] and text.count('process has finished cleanly')==2 and 'process has died' not in text
    assert result['invalid']==0 and result['commands']>100 and result['motor_samples']>100
    assert result['max_command_speed']<=2+1e-5 and result['max_command_acceleration']<=2+1e-5
    assert result['max_command_fd_acceleration']<=2+1e-3
    assert result['max_actual_speed']<=2+1e-3 and result['max_actual_acceleration']<=2+1e-3
    assert result['max_tracking_error']<0.3 and result['final_tracking_error']<0.05 and result['final_actual_speed']<0.03
    if args.stop=='none':
        assert result['ready'] and math.dist(last_command[1],(1,0,3))<1e-6
        assert result['clock_pause_samples']>5 and result['clock_pause_position_drift']<1e-9
    else:
        assert result['stop_requested'] and result['stopped'] and not result['ready']
        assert result['reference_stop_distance']<=result['ideal_stop_distance_at_request']+0.05

if __name__=='__main__': main()
