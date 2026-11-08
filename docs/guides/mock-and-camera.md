# The mock arm and the camera alone

Two ways to work without the arm and without the simulator.

## The mock arm

```bash
./scripts/run/mock.sh moveit
./scripts/run/mock.sh jog
```

`ros2_control`'s mock hardware follows every command at once, so MoveIt plans and executes in RViz,
and `jog` adds Servo with the keyboard jog (`piper_teleop`'s README has the keys). Nothing needs a
GPU; it is the quickest check of a change to the description, the controllers or MoveIt.

## The wrist camera

```bash
sudo ./scripts/setup-host.sh dds
./scripts/run/camera.sh view
```

The Percipio GM461-E1 on the wired NIC at `192.168.10.200` by default (`device_ip:=` changes it),
with its colour, depth and point cloud in RViz. On its own the driver publishes its own frames;
with the arm's stack it runs with `publish_tf:=false` and the robot description places it on the
wrist. Modes: colour 1280x960, 640x480 (the default) or 320x240; depth 1280x800, 640x400 or
320x200; 8 to 30 frames a second.
