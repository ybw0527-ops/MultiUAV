# Native SO3 execution model

`falcon_so3_simulator` ports the upstream EPEE quadrotor dynamics to ROS 2 and
provides a native controller wrapper. Position commands produce world forces,
desired attitudes and four rotor inputs. Position, velocity, attitude, angular
velocity and rotor RPM evolve through RK4 integration with gravity, drag,
gyroscopic coupling and the original 1/30 s motor time constant. Desired poses
are never assigned directly to the plant state.

The native controller retains upstream position/velocity PD, acceleration and
gravity feedforward, SO(3) attitude error, body-rate damping and plus-configuration
motor allocation. It omits the upstream adaptive acceleration-error correction
whose IMU-frame semantics were inconsistent. The tilt limit is actually 45 degrees
(upstream's comment said 45 but its code used pi/2). Numerical parity with that
buggy branch is not claimed. Controller mass is the same as the plant mass.

The port initializes acceleration and external moment, removes unused reads of
uninitialized blade-flow variables, fixes the Cholesky rotation projection to
use the transpose factor, and rejects nonfinite integration state/input instead
of silently replacing NaNs. Integration uses measured ROS elapsed time, subdivided
into at most 1 ms steps by default. A paused clock does not advance the plant;
backward jumps or gaps above `so3_simulator.maximum_clock_catchup` (2 s by
default) fail explicitly. Supported catch-up replays timestamped command events
and the 0.5 s command watchdog in chronological order. Future commands do not
act early, and late commands do not rewrite already integrated history.

Between command events, the reference advances from its received position,
velocity and acceleration, with prediction speed limited to 1.7 m/s by default.
Keeping an old position beside a nonzero velocity during a long command gap
caused large tracking errors. After 0.5 s without a new input, the predicted
reference brakes at 1.2 m/s² to a stationary hold; it does not jump backward to
the last received position. These are reference operations, still tracked by
the force controller and rotor dynamics.

The default total feedback-plus-feedforward net acceleration request is bounded
to 1.5 m/s² before converting force to attitude, leaving feedback margin above
the 1.2 m/s² reference bound. Rotor allocation scales torque
requests together to preserve collective thrust, retaining 50% collective headroom
on each rotor to limit thrust loss during motor lag. Independent rotor clipping
previously raised total thrust during saturated yaw requests. These changes act
on controller inputs; actual state and observed acceleration are never clipped.

## Launch and validation

Build the workspace, source ROS 2 and the workspace, then select the model:

```bash
ros2 launch falcon_map_render simulation.launch.py execution_model:=so3
```

The default remains `execution_model:=kinematic`. The SO3 preset applies reference
velocity/acceleration scales of 0.85/0.60, giving 1.7 m/s and 1.2 m/s² reference
bounds for the configured 2 m/s and 2 m/s² physical observation gates. These are
validated-model margins, not universal guarantees for different gains or airframes.
An unscaled initialization run reached approximately 2.50 m/s² actual acceleration
despite a 2 m/s² reference bound; that observation is retained in the validation
record. Actual odometry is checked independently of command derivatives.

On this WSL host, some Fast DDS runs discovered endpoints but delivered no data.
This was not attributed to a proven root cause. The tracked baseline uses the
installed Cyclone DDS backend. Full depth-image transport additionally uses small
UDP payloads and DDS fragments, as supported by the
[Cyclone DDS configuration reference](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html).
The profile is opt-in and changes no global middleware settings:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="$(ros2 pkg prefix falcon_so3_simulator)/share/falcon_so3_simulator/config/cyclone_loopback.xml"
cd ros2_ws
python3 src/falcon_fast_planner/test/initialization_tracking_regression.py --stop none
python3 src/falcon_fast_planner/test/initialization_tracking_regression.py --stop vertical
python3 src/falcon_fast_planner/test/initialization_tracking_regression.py --stop horizontal
python3 src/falcon_fast_planner/test/initialization_tracking_regression.py --stop none --delay-model-clock
python3 src/falcon_fast_planner/test/initialization_tracking_regression.py --stop none --delay-global-clock
python3 src/falcon_map_render/test/autonomous_regression.py --map all --execution-model so3 --duration 60
```

The initialization test gives node discovery a separate startup budget and only
starts its motion clock once commands arrive. It verifies vertical/horizontal
mid-maneuver braking, actual settling, finite RPM, command limits, physical-state
speed/acceleration gates, and a pause with zero command-position drift. Cancelling
initialization holds the stopped position and keeps initialization readiness false;
the FSM does not automatically explore from an aborted preflight maneuver.

The integrated execution milestone records 79 passing colcon checks, all six
60-second SO3 exploration runs, normal/vertical-abort/horizontal-abort startup,
separate 0.65 s model-clock and global-clock interruptions, and a 60-second
kinematic tunnel compatibility run. The six SO3 runs completed 17 stop episodes;
maximum integrated speed/acceleration were 1.683 m/s and 1.561 m/s², maximum
sampled tracking error was 0.0814 m, and minimum source-geometry clearance lower
bound was 0.414 m for a 0.2 m radius. All four nodes exited cleanly in every map.
See `test/execution_run_summary.json` for metrics, earlier rejected attempts,
commands and limitations. These are bounded runs, not completed coverage missions.

## Interfaces and limits

Inputs: `command` (PositionCommand), `force_disturbance` and `moment_disturbance`
(Vector3, world force and body moment). Outputs: `odometry`, `imu`, `motor_rpm`
and diagnostic `so3_cmd`. Additional outputs are `state_trace` (actual integrated
poses at intervals no greater than 11 ms under the default integration settings,
including catch-up) and `integration_status` (Float64MultiArray: maximum clock
interval, catch-up count, maximum input lateness, maximum integrated speed and
maximum integrated acceleration). The last two metrics include every integration
substep from startup, so sparse odometry cannot hide a transient peak. The graph
remaps these to `/uav_simulator/...` topics.
Odometry preserves FALCON's world-linear/body-angular velocity convention. IMU
specific force is expressed in the body frame with gravity included correctly.

The plant starts at hover RPM and has a point-floor clamp at z=0; rotor arming,
ground contact, geometry collision response and calibrated aerodynamics are not
modeled. Geometry clearance is checked separately on actual recorded positions.
Jerk/thrust/roll/pitch hard actuator constraints, angular tracking bounds, changed
airframes/gains, unforeseen latency/disturbances and complete six-map tracked
missions remain unvalidated. Observed finite-duration runs are not hardware flight
or emergency-stop certification.
