# piper_lerobot

[LeRobot](https://github.com/huggingface/lerobot) (Hugging Face) on the Piper's stack: recording
datasets while an operator drives the arm, and running LeRobot policies on it. LeRobot pins numpy
below 2.3, which has no wheels for Lyrical's Python 3.14, so it runs in its own Python 3.12
environment (`/opt/lerobot-venv`, from `docker/dev/lerobot/`) and reaches the stack through a
bridge node over ZMQ instead of rclpy.

- `lerobot_bridge` (C++) serves the arm's joint positions, efforts and commanded positions (the
  trajectory controller's), its cameras' colour images (`cameras`, `camera_topics`: the wrist and
  the overhead camera) and an episode marker (`/lerobot/episode`, latched `std_msgs/String`), and
  turns joint targets into MoveIt Servo joint jogs (so Servo checks collisions and joint limits)
  and gripper goals.
- `lerobot/lerobot_robot_piper`: LeRobot's `piper_ros` robot. Observations: `joint1.pos` to
  `joint6.pos` (rad), `gripper.pos` (one finger's travel, m), with `--robot.effort=true` each
  joint's effort, and the cameras of `--robot.images` (default `wrist:480x640`; an image served
  larger is scaled down to its size, e.g. `overhead:360x640`); actions: the same joint positions
  as targets. `--robot.passive=true` sends no actions.
- `lerobot/lerobot_teleoperator_piper`: LeRobot's `piper_stack` teleoperator, the arm's commanded
  positions as actions, so recording takes what the stack's own teleoperation (the keyboard jog)
  commands. The gripper's action is its measured opening: its controller publishes no command.
- `lerobot/piper_lerobot.py`: LeRobot's command line with the two plugins registered (LeRobot finds
  plugins only as installed packages; these run from the source).

- `lerobot/record_episodes.py`: records LeRobot episodes cut by the episode marker
  (`record:<n>:<task>`, then `keep:<n>` or `discard:<n>`; `end` stops it).
- `scripts/expert_episodes.py` (`ros2 run`): runs a contact task's stage again and again in the
  simulator as episodes: the arm to a random SRDF pose, the simulator's `sim/reset_task` to just
  before the stage, the marker, the skills' `run_stage`, and `keep` when it succeeded.

Python for the plugins and the recorder because LeRobot is Python; for `expert_episodes.py`
because it sequences services and actions a few times a minute.

## Run

The stack with Servo, then the bridge (in the sim, on its clock):

```bash
ros2 launch piper_bringup bringup.launch.py hardware:=isaac motion:=true moveit:=true servo:=true
ros2 launch piper_lerobot lerobot_bridge.launch.py use_sim_time:=true
```

Recording: drive the arm with `piper_teleop`'s keyboard jog while LeRobot records; datasets stay
local (`HF_HUB_OFFLINE=1`, nothing goes to the Hugging Face Hub):

```bash
cd /piper-stack/ros_ws/src/piper_lerobot/lerobot
HF_HUB_OFFLINE=1 PYTHONPATH=$PWD /opt/lerobot-venv/bin/python piper_lerobot.py record \
    --robot.type=piper_ros --robot.passive=true --teleop.type=piper_stack \
    --dataset.repo_id=local/pick_cube --dataset.root=/piper-stack/data/lerobot/pick_cube \
    --dataset.push_to_hub=false --dataset.num_episodes=10 --dataset.episode_time_s=30 \
    --dataset.fps=15 --dataset.single_task="Pick up the red cube" --display_data=false
```

Scripted demonstrations of a contact task's stage, with the simulator (`TASK=assembly`, a
`TASK_SEED` for the fixture's placement), the stack with the task (`task_file:=...`), the skills
with it and the bridge up:

```bash
cd /piper-stack/ros_ws/src/piper_lerobot/lerobot
HF_HUB_OFFLINE=1 PYTHONPATH=$PWD /opt/lerobot-venv/bin/python record_episodes.py \
    --root /piper-stack/data/lerobot/assembly_ring --repo-id local/assembly_ring \
    --images wrist:480x640,overhead:360x640 --effort --episodes 20 &
ros2 run piper_lerobot expert_episodes.py --stage ring --episodes 20 \
    --task "Put the ring on the post" --ros-args -p use_sim_time:=true
```

With `--policy <checkpoint>/pretrained_model` (and the stack's `servo:=true`),
`expert_episodes.py` evaluates a trained policy on the stage instead: each episode the policy runs
through `lerobot-rollout` for `--rollout-s`, the stage's contacts allowed in MoveIt's scene, and it
counts as a success when the part ends within 3 mm of its goal on its mate (the simulator's ground
truth).

A LeRobot policy drives the arm through `piper_ros` without `passive`: its joint targets go
through Servo, so a target the arm cannot reach without a collision or past a joint limit stops
it there. `piper_lerobot.py rollout` and `piper_lerobot.py train` hand over to LeRobot's own
`lerobot-rollout` and `lerobot-train`; neither has run on this robot yet (no policy is trained).

## The bridge

| Topic, service or action | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | In |
| `/arm_controller/controller_state` | `control_msgs/JointTrajectoryControllerState` | In: the commanded positions |
| `/wrist_camera/color/image_raw` | `sensor_msgs/Image` (rgb8 or bgr8) | In |
| `/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | Out, at 100 Hz while targets come |
| `/servo_node/switch_command_type` | `moveit_msgs/ServoCommandType` | Called: joint jog |
| `/gripper_controller/gripper_cmd` | `control_msgs/ParallelGripperCommand` | Goals |

The ZMQ protocol (`include/piper_lerobot/bridge_protocol.hpp`) is request-reply on
`tcp://127.0.0.1:5590`: `observe`, `act` with the seven targets, `release`. A joint's jog velocity
is `jog_gain` (2 /s) times its distance to the target, capped at `max_jog_velocity` (0.5 rad/s);
when no target has come for `target_timeout_s` (0.5 s) the bridge stops jogging and Servo ramps
the arm down. Parameters: `config/lerobot_bridge.yaml`.

## Tests

```bash
./scripts/manage.sh test piper_lerobot
```

`test_bridge_protocol` checks the frames' encoding and decoding (lengths, non-finite values) and
the jog law; `test_lerobot_plugins` (pytest, in LeRobot's environment) runs the robot and the
teleoperator against a stub of the bridge: observations, the order of the targets, passive mode,
release on disconnect, a refused request and a bridge that does not answer.
