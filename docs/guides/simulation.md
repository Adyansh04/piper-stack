# Simulation

The stack against the Piper in Isaac Sim 6.2: the arm on a table with four 5 cm cubes or a task's
parts, the wrist camera rendered with the real camera's resolution and focal length, PhysX or
Newton at 200 Hz.

## Once

Build the simulator's image (Isaac Sim's base image is large; the first build takes a while):

```bash
docker compose --profile sim build sim
```

## Run

```bash
./scripts/run/sim.sh moveit
```

| Variant | Starts |
|---|---|
| `arm` | The simulator, then the bringup on `hardware:=isaac` with RViz |
| `moveit` | `arm`, plus MoveIt with its RViz panel |
| `jog` | `moveit`, plus Servo and the keyboard jog |
| `mission` | `moveit`, plus SAM 3.1, GraspGenX, the skills, the behaviour tree executor and a pane that sends a `PickAndPlace` run on Enter |
| `shadow` | The simulator alone, as the digital twin of a stack on the real (or mock) arm |
| `stop` | Ends the stack |

The simulator runs headless in its own container; the stack waits for its `/clock`. The first
start compiles shaders for a few minutes.

## Options

Environment variables before the command:

- `HEADLESS=false`: Isaac Sim's window on this PC's display (start it from the desktop, not over
  SSH).
- `LIVESTREAM=true`: Isaac Sim's UI over WebRTC instead, for a simulator on a GPU server: connect
  NVIDIA's Isaac Sim WebRTC Streaming Client to it (49100/tcp, 47998/udp). No authentication: keep
  it on a trusted network.
- `PHYSICS_ENGINE=newton`: Newton (MuJoCo-Warp with Newton's own signed-distance contacts, 3
  solver steps per physics step) instead of PhysX. The contact tasks need it: their threads and the
  USB-C fit collide through each mesh's signed distance field.
- `TASK=assembly` (or another folder under `tasks/`): that task's parts on the table at their start
  poses instead of the cubes ([tasks/README.md](../../tasks/README.md)). With MoveIt, the run
  script also starts `piper_skills`' `task_scene`, which puts the parts in MoveIt's planning scene.

`sim/config/sim.yaml` holds the rates, the world (table, cubes and their friction, the task), the
cameras, Newton's solver settings and the joint drives, with a comment on each value.

## What it publishes

- `/clock`, and `/isaac_joint_states` for `ros2_control` (it takes `/isaac_joint_commands`).
- `/wrist_camera/color/image_raw`, `/wrist_camera/depth/image_raw` (32FC1 metres, aligned to colour)
  and their `camera_info`, as Percipio's driver names them.
- `/overhead_camera/color/image_raw` and its `camera_info` (1280 x 720, 70 degree field of view),
  as a USB camera driver names them: the overhead RGB camera, 1 m above the table looking down, in
  `overhead_camera_color_optical_frame` (the robot description holds its pose).
- `/sim/objects`: the cubes' or the task parts' poses in `world` (TFMessage, the child frame is the
  object's name), every 0.5 s of simulation time, static parts (a task's fixture) included: the
  ground truth for tests, scripted experts and MoveIt's planning scene.
