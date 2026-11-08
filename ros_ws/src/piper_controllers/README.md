# piper_controllers

Trained policies on the Piper. `policy_runner` moves the arm with a reach policy (ONNX, from the
Isaac Lab task in [`lab/`](../../../lab/README.md)) through MoveIt Servo: the tool goes to a
target pose, the joints moving under velocity and acceleration caps, and Servo checks every
command for collisions, joint limits and singularities.

## Run

Bringup starts it, inactive, when given a policy (`policy.onnx`, with its `policy.onnx.data`
beside it) and Servo:

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=isaac motion:=true moveit:=true servo:=true policy:=/piper-stack/data/policies/reach/policy.onnx
ros2 service call /policy_runner/set_active std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /policy_runner/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.07}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 service call /policy_runner/set_active std_srvs/srv/SetBool "{data: false}"
```

That target is 7 cm above the table, 25 cm in front of the base, the tool pointing down. The
policy was trained from SRDF `ready` with targets 15 to 30 cm in front, 15 cm to either side and 3
to 10 cm above the table, tilted at most 15 degrees: move to `ready` first, and keep targets in
that box. On the real arm, the first run of a policy goes at reduced caps, with the user's go.

## How it moves the arm

- `~/set_active` true switches Servo to joint jogs and starts from the measured positions, at rest;
  it is refused before every joint has a state on `/joint_states` or while Servo is not running.
  The joints hold until a target arrives; a target sent while inactive is ignored.
- At `policy_rate_hz` (50 Hz, every fourth of the 200 Hz ramp steps) the policy reads joints 1 to
  6 (positions in rad, velocities in rad/s) and the target (position in m, then the quaternion x,
  y, z, w, in `base_link`), and gives one action per joint.
- Each action sets the joint's wanted position: `action_offset` plus `action_scale` times the
  action, inside the URDF's limits.
- Every ramp step `JointRamp` moves each joint's command towards it: the commanded velocity is the
  fastest from which the joint still stops at the wanted position, at most `max_velocity`, and
  changes by at most `max_acceleration`. The jog sent to Servo, stamped on the ROS clock, is that
  velocity plus `tracking_gain` (5/s) times the gap between the command and the measured
  position, at most `max_velocity`: Servo moves on from the measured state, so without it a
  tracking lag would stay as a shortfall.
- A target in another frame, or with a non-finite position or no orientation, is ignored. If
  inference fails, or an action is not finite, the joints come to rest where they are.
  `~/set_active` false brings them to rest under the caps, then the runner stops sending.
- The Isaac Lab task trains with the same law (`lab/piper_lab/actions.py`) and reads its values
  from this node's parameters, so the two never disagree.

## Topics and services

| Name | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | In, sensor data QoS |
| `~/target_pose` | `geometry_msgs/PoseStamped` | In, reliable, depth 1 |
| `/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | Out, at `jog_rate_hz` while active |
| `~/set_active` | `std_srvs/SetBool` | Service |
| `/servo_node/switch_command_type` | `moveit_msgs/ServoCommandType` | Called on activation |

## Parameters

In `config/policy_runner.yaml`: `joints`, `model_path` (set by the launch), `frame`
(`base_link`), `policy_rate_hz` (50), `jog_rate_hz` (200, the training's physics rate),
`action_offset` (`ready`), `action_scale` (0.5 rad), `max_velocity` (1 rad/s, a third of the
firmware's cap), `max_acceleration` (0.5 rad/s^2, the firmware's cap), `tracking_gain` (5/s), and
Servo's topic and service. The joint limits come from `robot_description` (the launch expands
`piper_description`). A policy works only with the values it was trained with: retrain after
changing any of them.

## Dependencies

ONNX Runtime 1.24 (CPU) from `onnxruntime_vendor`. Inference runs single-threaded on buffers bound
at start.

## Tests

```bash
./scripts/manage.sh test piper_controllers
```

`test_joint_ramp` checks the motion law step by step against the training action, its caps, that
it arrives without overshooting by more than about 2 mrad, the joint limits, stopping on a
non-finite action, the velocities it reports, and that it refuses inconsistent limits and caps.
