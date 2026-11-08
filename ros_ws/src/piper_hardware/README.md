# piper_hardware

The `ros2_control` system of the real Piper over CAN (`piper_hardware/PiperSystem`), on top of
`piper_driver`. It is the only part of the stack that sends motor commands to the arm. The
simulator and the mock arm use other systems behind the same interfaces (`piper_description`'s
`hardware` argument), so the controllers and MoveIt do not change between them.

## Run

Through the bringup, never on its own:

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=can rviz:=true
ros2 launch piper_bringup bringup.launch.py hardware:=can enable:=true motion:=true
```

The first is read-only: activating leaves the motors as they are and sends nothing. With
`enable:=true` activation sets the collision level and payload, the motion mode and a hold target
at the measured pose, then enables the motors, so they come on holding where the arm is. Every
motion on the real arm needs the operator at the arm.

## Interfaces

Per joint (`joint1` to `joint6`, and the gripper's `joint7`): state `position`, `velocity`,
`effort`; command `position`, plus `velocity` and `effort` in MIT mode (a velocity target and a
feed-forward torque).

## Parameters

In the robot description (`piper_description/urdf/piper.ros2_control.xacro`), most from the
bringup's arguments:

| Parameter | Meaning |
|---|---|
| `can_interface` | The arm's CAN interface (`can_follower`, 1 Mbit/s) |
| `enable_on_activate` | `false`: read-only (`enable:=` of the bringup) |
| `motion_mode` | `joint` (MOVE J, the firmware interpolates), `cpv` (MOVE CPV, streaming), `mit` (per-joint impedance) |
| `speed_percent` | The firmware's speed scaling for MOVE J, percent of 3 rad/s |
| `mit_kp`, `mit_kd` | MIT gains, N m/rad and N m s/rad; `mit_max_torque_nm`, `mit_max_velocity_rad_s` clamp the commands |
| `collision_level` | 0 (off) to 8 (most sensitive), set at activation; -1 keeps the firmware's |
| `end_load` | Payload class set at activation: empty, half or full; nothing keeps the firmware's |
| `feedback_timeout_ms` | No new joint frame for this long is an error (the arm sends them at 200 Hz) |
| `gripper_effort_nm` | Grip torque sent with gripper targets |

A firmware fault in the arm's status, or feedback that stops, is an error the controller manager
reports; the plugin never disables the motors, and deactivating sends nothing, so the arm keeps
holding.

## Tests

Its behaviour against the fake arm on `vcan0` (every motion mode through the controllers) is
checked with the bringup on `hardware:=can can_interface:=vcan0`; `piper_driver`'s tests cover the
protocol underneath.
