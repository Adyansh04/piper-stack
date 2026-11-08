# piper_controllers

`ros2_control` controllers of the Piper. `piper_controllers/PolicyController` runs a trained reach
policy (ONNX, from the Isaac Lab task in [`lab/`](../../../lab/README.md)) inside the control loop:
the tool goes to a target pose, the joints moving under velocity and acceleration caps.

## Run

Bringup loads it inactive when given a policy (`policy.onnx`, with its `policy.onnx.data` beside
it). It claims joints 1 to 6, so it takes over from `arm_controller`:

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=isaac motion:=true policy:=/piper-stack/data/policies/reach/policy.onnx
ros2 control switch_controllers --deactivate arm_controller --activate policy_controller
ros2 topic pub --once /policy_controller/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.07}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 control switch_controllers --deactivate policy_controller --activate arm_controller
```

That target is 7 cm above the table, 25 cm in front of the base, the tool pointing down. The
policy was trained from SRDF `ready` with targets 15 to 30 cm in front, 15 cm to either side and 3
to 10 cm above the table, tilted at most 15 degrees: move to `ready` first, and keep targets in
that box. On the real arm, the first run of a policy goes at reduced caps, with the user's go.

## How it moves the arm

- At `policy_rate_hz` (50 Hz, every fourth cycle of the 200 Hz loop) the policy reads joints 1 to
  6 (positions in rad, velocities in rad/s) and the target (position in m, then the quaternion x,
  y, z, w, in `base_link`), and gives one action per joint.
- Each action sets the joint's wanted position: `action_offset` plus `action_scale` times the
  action, inside the URDF's limits.
- Every cycle `JointRamp` moves each position command towards it: the commanded velocity is the
  fastest from which the joint still stops at the wanted position, at most `max_velocity`, and
  changes by at most `max_acceleration`. On activation the commands start at the measured
  positions, at rest.
- The joints hold until a target arrives after activation. A target in another frame, or with a
  non-finite position or no orientation, is ignored. If inference fails, or an action is not
  finite, the joints come to rest where they are.
- The Isaac Lab task trains with the same law (`lab/piper_lab/actions.py`) and reads its values
  from this controller's parameters, so the two never disagree.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `~/target_pose` | `geometry_msgs/PoseStamped` | In, reliable, depth 1 |

It claims the `position` command and the `position` and `velocity` state interfaces of `joints`.

## Parameters

In `piper_bringup/config/controllers.yaml`, section `policy_controller`: `joints`, `model_path`
(set by bringup's `policy:=`), `frame` (`base_link`), `policy_rate_hz` (50), `action_offset`
(`ready`), `action_scale` (0.5 rad), `max_velocity` (1 rad/s, a third of the firmware's cap) and
`max_acceleration` (0.5 rad/s^2, the firmware's cap). A policy works only with the values it was
trained with: retrain after changing any of them.

## Dependencies

ONNX Runtime 1.24 (CPU) from `onnxruntime_vendor`. Inference runs in the control loop's thread,
single-threaded, on buffers bound at configure.

## Tests

```bash
./scripts/manage.sh test piper_controllers
```

`test_joint_ramp` checks the motion law step by step against the training action, its caps, that
it arrives without overshooting by more than about 2 mrad, the joint limits, stopping on a
non-finite action, and that it refuses inconsistent limits and caps.
