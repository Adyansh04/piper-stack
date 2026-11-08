# piper_driver

The AgileX Piper over SocketCAN, without ROS: the protocol V2 codec, continuous feedback decoding,
the commands, and a fake arm that answers like the firmware on a virtual CAN interface.
`piper_hardware` runs it inside `ros2_control`; nothing else sends motor commands while that runs.

## Library

`piper_driver::Arm` opens one CAN interface and decodes the arm's feedback on a thread of its own;
`state()` returns the newest joint positions, velocities, efforts, gripper and status without
blocking. Its commands: enable and disable, the control and motion mode (MOVE J, MOVE CPV, MIT),
joint and gripper targets, MIT commands with per-joint gains (clamped), the collision protection
level per joint, the payload class, and the firmware's emergency stop and resume.
`piper_driver::FakeArm` plays the firmware on a virtual interface for tests.

The protocol follows the firmware S-V1.8-2 (protocol V2), with
[piper_sdk](https://github.com/agilexrobotics/piper_sdk) by AgileX Robotics as the reference.

## Tools

Run in the dev container, after `scripts/setup-host.sh can` (the real arm on `can_follower`) or
`scripts/setup-host.sh vcan` (`vcan0`):

```bash
ros2 run piper_driver piper_monitor can_follower
ros2 run piper_driver piper_fake_arm vcan0
```

- `piper_monitor [interface] [seconds]` prints the decoded feedback ten times a second. It never
  sends a frame.
- `piper_fake_arm [interface]` runs the fake firmware until Ctrl-C.
- `piper_disable <interface> --yes` disables every motor: a raised arm falls. Only with the arm at
  rest or held, and never while the stack runs.

## Tests

```bash
./scripts/manage.sh test piper_driver
```

`test_protocol` covers the codec (every frame the stack sends and decodes), `test_triple_buffer`
the lock-free hand-over from the CAN thread, and `test_arm_vcan` the driver against the fake arm on
`vcan0`: feedback, holding while disabled, the emergency stop, non-finite targets refused, another
sender's command frames counted, the firmware text, MIT commands, collision levels and payload
(skipped when `vcan0` is down).
