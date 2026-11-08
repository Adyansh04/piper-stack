# piper_moveit_config

MoveIt 2 for the Piper: the semantic description (planning groups, named states, the collision
matrix with the camera mount and camera), pick_ik for inverse kinematics, OMPL and Pilz for
planning, MoveIt Servo for jogging, and an RViz view.

## Run

`piper_bringup` starts it with `moveit:=true`:

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=mock motion:=true moveit:=true rviz:=true
```

On its own, next to a running bringup:

```bash
ros2 launch piper_moveit_config moveit.launch.py servo:=true rviz:=true
```

## Groups and states

- `arm`: `joint1` to `joint6`, tip `tcp`. States: `rest` (folded, the motors' power-off pose),
  `ready` (the wrist camera looks straight down at the work area from 0.31 m), `view_left` and
  `view_right` (`ready` turned 0.25 rad about joint 1, for the skills' side views).
- `gripper`: `joint7` (`joint8` mimics it). States: `open`, `closed`.

## Planning

- OMPL for free moves, Pilz for PTP and LIN (the skills' straight approaches and lifts); both
  under the joint limits of `config/joint_limits.yaml` (3 rad/s, the firmware's top speed, and
  0.5 rad/s^2).
- pick_ik (`config/kinematics.yaml`) for planning and `compute_ik`, with 1e-5 m tolerances: Pilz
  LIN differentiates successive solutions. Servo uses KDL (`config/servo_kinematics.yaml`) for its
  small steps.
- Servo (`config/servo.yaml`): twists and joint jogs from `/servo_node/delta_twist_cmds` and
  `/servo_node/delta_joint_cmds` (`piper_teleop`), a Butterworth filter, trajectories to the arm
  controller.

## Tests

None of its own: the MoveIt checks run against the mock stack and Isaac Sim (OMPL, Pilz LIN and
PTP, pick_ik goals, the gripper, Servo).
