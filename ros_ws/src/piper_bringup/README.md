# piper_bringup

Starts the Piper's stack: `robot_state_publisher`, the `controller_manager` with the arm and
gripper controllers, MoveIt, and the wrist camera. One launch file serves the mock arm, the real
arm over CAN and the arm in Isaac Sim, chosen by `hardware`. It also records bags of the stack's
topics.

## Run

In the dev container (`./scripts/manage.sh exec`), with the workspace built:

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=mock motion:=true moveit:=true rviz:=true
```

The run scripts open the usual combinations in tilix panes, from the repository root on the host:
`./scripts/run/mock.sh`, `./scripts/run/sim.sh` and `./scripts/run/real.sh` (each prints its
variants with `--help`, and `--print` lists the commands instead of opening them).

## Launch files

| File | Starts |
|---|---|
| `bringup.launch.py` | The arm: description, `ros2_control`, controllers, MoveIt, Servo, RViz |
| `camera.launch.py` | The wrist camera through Percipio's driver, topics under `/wrist_camera` |
| `record.launch.py` | A rosbag2 recorder of one profile, into a new MCAP bag |

`bringup.launch.py`'s arguments (`--show-args` lists them all):

- `hardware`: `mock` (no arm), `can` (the real arm over `can_interface`) or `isaac` (Isaac Sim's
  joint topics, on its `/clock`).
- `enable`: `hardware:=can` only; `false` (the default) sends nothing to the arm.
- `motion`: also start the arm and gripper controllers; `moveit`, `servo`, `rviz` add those.
- `motion_mode`: `joint` (MOVE J), `cpv` or `mit` on the real arm; `speed_percent` scales MOVE J.
- `policy`: a trained reach policy (`policy.onnx`) for `piper_controllers`' policy runner, which
  moves the arm through MoveIt Servo (needs `moveit:=true servo:=true`); it starts inactive (see
  its README).

## Recording

```bash
ros2 launch piper_bringup record.launch.py profile:=arm
ros2 launch piper_bringup record.launch.py profile:=mission use_sim_time:=true
```

The bag goes to `/piper-stack/data/bags/<profile>_<date>_<time>` (the checkout's `data/`, ignored
by git); Ctrl-C closes it. The profiles are in `config/record/`:

| Profile | Records |
|---|---|
| `arm` | Joint states, the arm controller's state and commands, `/controller_manager/statistics` (each controller's and the hardware's cycle times and periodicity), diagnostics, TF, the estimator's wrench and grasp, and in sim the Isaac joint topics and `/clock` |
| `mission` | `arm`, plus the behaviour tree's `/bt_executor/status`, the skills' and `run_tree` action goals, feedback and results, the camera's `camera_info` and the sim's `/sim/objects` |
| `camera` | The wrist camera's colour and depth images with `camera_info`, joint states and TF |

`ros2 bag play` replays a bag, and PlotJuggler 4 plots it: `plotjuggler4 -d <bag>/<file>.mcap`.

## Plots

PlotJuggler 4 is in the dev image with its ROS 2 topic subscriber, ROS message parser and MCAP
loader. `config/plotjuggler/arm.pj4.xml` has three tabs: the joints' positions, velocities and
efforts; the gripper; and the control loop (the hardware's read and write periodicity, which on
the real arm is the CAN feedback's period, and each controller's execution time).

```bash
plotjuggler4 -l "$(ros2 pkg prefix piper_bringup)/share/piper_bringup/config/plotjuggler/arm.pj4.xml"
```

Then stream with the ROS 2 topic subscriber, or open a bag with `-d <bag>/<file>.mcap`.

## Parameters

- `config/controllers.yaml`: the controller_manager's 200 Hz loop, the joint trajectory controller
  (goal time, stopped velocity) and the gripper controller (goal tolerance, stall detection).
- `config/record/*.yaml`: the recording profiles above.
- `config/plotjuggler/arm.pj4.xml`: the PlotJuggler 4 layout above.

## Tests

None of its own: the package holds launch files and configuration, and the packages it starts
carry the tests.
