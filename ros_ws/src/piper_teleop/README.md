# piper_teleop

Jogs the Piper from the keyboard through MoveIt Servo: the tool in Cartesian space, or one joint
at a time, and the gripper. The keys become twists and joint jogs on Servo's input topics, the
same ones a gamepad or SpaceMouse node would publish instead.

## Run

With the stack and Servo up (`./scripts/run/mock.sh jog` or `./scripts/run/sim.sh jog` open both),
in a terminal of the dev container:

```bash
ros2 run piper_teleop keyboard_jog --ros-args \
  --params-file "$(ros2 pkg prefix piper_teleop)/share/piper_teleop/config/keyboard_jog.yaml"
```

It reads keys from its terminal, so it runs with `ros2 run`, not from a launch file.

| Keys | Do |
|---|---|
| `w` `s`, `a` `d`, `r` `f` | Move the tool along x, y, z |
| `u` `o`, `i` `k`, `j` `l` | Turn the tool about x, y, z |
| `1` to `6`, then `+` `-` | Choose a joint, then jog it |
| `t` | Switch between the base frame and the tool frame |
| `g`, `h` | Open, close the gripper |
| `]`, `[` | Double, halve the speeds |
| `q` | Quit |

## Topics and actions

Out: `/servo_node/delta_twist_cmds` (`geometry_msgs/TwistStamped`) and
`/servo_node/delta_joint_cmds` (`control_msgs/JointJog`); it calls
`/servo_node/switch_command_type` and the gripper controller's `/gripper_controller/gripper_cmd`
action.

## Parameters

`config/keyboard_jog.yaml`: the speeds at x1 (0.05 m/s, 0.3 rad/s, joints 0.3 rad/s), the command
rate (50 Hz) and how long one key press keeps a command going (0.15 s), the frames, the topics and
the gripper's open and closed travel and effort.

## Tests

```bash
./scripts/manage.sh test piper_teleop
```

`test_key_map` checks what each key does.
