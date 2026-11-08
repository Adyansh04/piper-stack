# The real arm

Running the stack on the AgileX Piper over its USB-CAN adapter, with the wrist camera. Everything
runs on the PC next to the arm.

## Safety first

- Every motion needs the operator at the arm, with a hand ready to hold it. There is no hardware
  e-stop: anything that cuts motor torque (disabling the motors, the firmware's emergency stop) is
  announced first, and the arm is held while it happens.
- Never stop the stack or disable the motors with the arm raised: move it to `rest` first. When
  commands stop, the firmware keeps holding the last pose.
- The first run of any new motion goes at reduced speed (`speed_percent`, the skills'
  `motion.velocity_scale`).

## Once per PC

On the host:

```bash
sudo ./scripts/setup-host.sh can
sudo ./scripts/setup-host.sh dds
```

`can` names the adapter by its serial `can_follower` and brings it up at 1 Mbit/s on every plug-in;
`dds` raises the socket buffers the camera's images need.

## Read-only first

```bash
./scripts/run/real.sh arm
```

The bringup starts read-only: joint states, the controllers and RViz, nothing sent to the arm.
`piper_monitor` in its own pane prints the decoded feedback. Check that RViz shows the arm as it
stands.

## Moving it

```bash
./scripts/run/real.sh arm enable:=true
./scripts/run/real.sh camera enable:=true
./scripts/run/real.sh mission enable:=true
```

`enable:=true` turns the motors on holding the measured pose. `camera` adds the wrist camera;
`mission` adds MoveIt, SAM 3.1, GraspGenX, the skills and the behaviour tree executor, with a pane
that sends a `PickAndPlace` run only when Enter is pressed. Other bringup arguments pass through
(`motion_mode:=mit`, `speed_percent:=20`).

## Ending

Move the arm to `rest` (MoveIt's RViz panel, or the `ArmToPose` tree with `pose=rest`), then:

```bash
./scripts/run/real.sh stop
```

`ros2 run piper_driver piper_disable can_follower --yes` turns the motors off: a raised arm falls,
so only with it at rest or held.
