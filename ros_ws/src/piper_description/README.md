# piper_description

The AgileX Piper's robot description (xacro): the arm and its parallel gripper, the camera mount
and the Percipio GM461-E1 wrist camera on it, a tool frame `tcp`, and the `ros2_control` system
for the mock arm, the real arm over CAN or Isaac Sim.

## Use

`piper_bringup` expands it; by hand, in the dev container:

```bash
xacro "$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper.urdf.xacro" \
  hardware:=can can_interface:=can_follower > /tmp/piper.urdf
```

| Argument | Default | Meaning |
|---|---|---|
| `hardware` | `mock` | `mock`, `can` (`piper_hardware`) or `isaac` (Isaac Sim's joint topics) |
| `can_interface` | `can_follower` | CAN interface of the arm (`hardware:=can`) |
| `enable_on_activate` | `false` | `hardware:=can`: `true` enables the motors and sends commands |
| `speed_percent` | `10` | The firmware's MOVE J speed scaling |
| `motion_mode` | `joint` | `joint`, `cpv` or `mit` (MIT adds velocity and effort command interfaces) |
| `collision_level`, `end_load` | `-1`, empty | Set at activation; the defaults keep the firmware's |
| `wrist_camera` | `true` | The camera mount and the camera's frames |
| `prefix` | empty | Prefix of every link and joint name |

## Frames

`base_link` sits on the table top (the planning frame `world` is fixed to it). The gripper's
`tcp` is in the middle of the finger pads (`gripper_base` z 0.1295 m). The camera's frames are
named as Percipio's driver names them (`wrist_camera_link`, `wrist_camera_color_frame`,
`wrist_camera_color_optical_frame`, `wrist_camera_depth_frame`, `wrist_camera_depth_optical_frame`),
so the driver runs with `publish_tf:=false` next to `robot_state_publisher`. The lens offsets inside
the camera are estimates until the hand-eye calibration at the arm.

## Meshes

- `meshes/*.STL`: AgileX's collision meshes, from
  [piper_ros](https://github.com/agilexrobotics/piper_ros) (MIT).
- `meshes/visual/`: coloured visual meshes from
  [MuJoCo Menagerie's agilex_piper](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)
  (MIT), merged per link and colour.
- `meshes/camera_mount.stl`: the wrist camera mount, from its STEP model.

## Tests

```bash
./scripts/manage.sh test piper_description
```

The tests expand the description for each hardware (mock, CAN, Isaac Sim, and CAN in MIT mode
with a collision level and payload) and without the camera.
