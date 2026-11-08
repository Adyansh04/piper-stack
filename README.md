# piper-stack

A ROS 2 Lyrical stack for the AgileX Piper arm with a Percipio GM461-E1 wrist camera: a C++
`ros2_control` driver over CAN, Isaac Sim 6.2 for simulation and Isaac Lab 3.0 for training.
Everything runs in Docker. Early development.

## Run

On the host, from the repository root:

```bash
cp .env.example .env
./scripts/manage.sh start
./scripts/manage.sh build
./scripts/manage.sh test
./scripts/manage.sh exec
```

`start` builds the dev image (`docker/dev/Dockerfile`) on first use and starts the container;
`exec` opens a shell in it with the workspace sourced.

### In simulation

```bash
./scripts/run/sim.sh moveit
```

This starts Isaac Sim headless in the sim container (`docker/sim/Dockerfile`, built on first use
with `docker compose --profile sim build sim`) with the arm on a table, four 5 cm cubes and the
wrist camera, then the stack on `hardware:=isaac`, on the simulator's clock. The first start
compiles shaders for a few minutes. `sim/config/sim.yaml` sets the physics and render rates, the
world and the camera.

```bash
./scripts/run/sim.sh mission
```

adds SAM 3.1, GraspGenX, the pick and place skills (`piper_skills`) and the behaviour tree executor
(`piper_bt`), with a pane that sends a `PickAndPlace` run (the red cube to the table's left third)
each time Enter is pressed. `./scripts/run/real.sh mission enable:=true` does the same on the real
arm; any motion there needs the operator at the arm.

With a stack on the real arm running (`./scripts/run/real.sh arm`), `./scripts/run/sim.sh shadow`
starts Isaac Sim as its digital twin: the simulated arm follows `/joint_states`, publishes its own
state on `/shadow/joint_states` and renders its wrist camera under `/shadow/wrist_camera`, next to
the real camera's images. The twin publishes no `/clock` and has no cubes.

For a simulator on another machine (a GPU server), `LIVESTREAM=true ./scripts/run/sim.sh moveit`
streams Isaac Sim's UI over WebRTC instead of rendering to a window: connect NVIDIA's Isaac Sim
WebRTC Streaming Client to that machine (49100/tcp and 47998/udp). The stream has no
authentication, so keep it on a trusted network. On the real arm everything runs on the local PC.

### Recording and plots

```bash
ros2 launch piper_bringup record.launch.py profile:=mission
plotjuggler4
```

`record.launch.py` writes MCAP bags to `data/bags/` (profiles in `piper_bringup`'s README);
PlotJuggler 4, in the dev image with its ROS 2 extensions, plots live topics and bags.

## Credits

- [piper_sdk](https://github.com/agilexrobotics/piper_sdk) by AgileX Robotics (MIT): the CAN
  protocol reference for `piper_driver`.
- [piper_ros](https://github.com/agilexrobotics/piper_ros) by AgileX Robotics (MIT): the robot
  description and its collision meshes.
- [MuJoCo Menagerie's agilex_piper](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)
  by RosenYin (MIT): the coloured visual meshes, merged per link and colour.
- [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) and
  [Isaac Lab](https://github.com/isaac-sim/IsaacLab) by NVIDIA: the simulator and its training
  framework (`docker/sim`).
- [SAM 3.1](https://github.com/facebookresearch/sam3) by Meta (SAM License) and
  [GraspGenX](https://github.com/NVlabs/GraspGenX) by NVIDIA (Apache 2.0): segmentation and grasp
  generation in `piper_perception`.
