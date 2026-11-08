# lab: Isaac Lab tasks for the Piper

Isaac Lab 3.0 tasks on the same arm the simulator runs, and the policies they train for the stack.
One task so far, `Piper-Reach`: the tool centre point (`tcp`) to a target pose over the table. Its
exported policy runs in the stack as `piper_controllers`' policy controller. Python because Isaac
Lab is Python.

## Train and export

In the sim container (`docker compose --profile sim up -d sim`), which carries Isaac Lab at
`/isaaclab`. Runs go to `data/lab/logs/rsl_rl/piper_reach/<date>_<time>/`. Training is headless
(Isaac Lab 3.0 opens a window only with `--viz kit`); stop the simulator first, one GPU job at a
time.

```bash
mkdir -p data/lab
docker exec -w /piper-stack/data/lab -e PYTHONPATH=/piper-stack/lab piper-sim \
    /isaaclab/isaaclab.sh -p /isaaclab/scripts/reinforcement_learning/train.py \
    --rl_library rsl_rl --task Piper-Reach --external_callback piper_lab.register \
    --num_envs 2048 --max_iterations 1000
```

Play loads the newest run's last checkpoint, exports it next to the run as
`exported/policy.onnx` (with its weights in `policy.onnx.data`) and then keeps playing; Ctrl-C it
once the file is there, or add `--viz kit` to watch the arms:

```bash
docker exec -w /piper-stack/data/lab -e PYTHONPATH=/piper-stack/lab piper-sim \
    /isaaclab/isaaclab.sh -p /isaaclab/scripts/reinforcement_learning/play.py \
    --rl_library rsl_rl --task Piper-Reach --external_callback piper_lab.register --num_envs 16
```

Then run it in the stack with `bringup.launch.py policy:=<run>/exported/policy.onnx` (see
`piper_controllers`' README). On an RTX 5090, 2048 arms for 1000 iterations take 22 minutes. That
policy brings the TCP within about 3 cm of its targets in Isaac Sim, through the stack's policy
controller, as in training; its orientation is still loose (about 40 degrees off), which longer
training or a heavier orientation reward should improve.

## The reach task

- The arm: the simulator's own robot USD (`sim/robot.py` converts the URDF, with `sim.yaml`'s joint
  drives) on its table, physics at 200 Hz. Each environment's arm is parsed on its own: the
  importer anchors the base with a fixed joint to the robot's own prim, which Isaac Lab's
  replication of the first environment does not carry over.
- Observations: joints 1 to 6 (positions in rad, velocities in rad/s, with noise of 5 mrad and
  0.02 rad/s, and up to one policy step late) and the target in the base frame (position in m,
  then the quaternion x, y, z, w).
- Actions, at 50 Hz: each joint's wanted position, which `piper_lab.actions` approaches under the
  velocity and acceleration caps, as the policy controller does on the stack. The joints, offsets,
  scale, caps and rate come from the policy controller's section of
  `piper_bringup/config/controllers.yaml`, so a trained policy always matches its controller.
- Targets: 15 to 30 cm in front of the base, up to 15 cm to either side, 3 to 10 cm above the
  table, the tool pointing down within 15 degrees and turned up to 90 degrees about the vertical;
  two targets in an episode of 12 s, starting from `ready`.
- Rewards: Isaac Lab's deployed reach task's keypoint tracking (keypoints 15 cm from the tool), and
  small penalties on the actions and their changes.
- Randomisation: the joints' start (0.125 rad around `ready`), the drives' stiffness (0.9 to 1.1
  times) and damping (0.75 to 1.5 times), joint friction, and the links' masses (0.8 to 1.2 times).

## Files

| File | Holds |
|---|---|
| `piper_lab/__init__.py` | `register()`, the gym registration that `--external_callback` calls |
| `piper_lab/reach_env_cfg.py` | The reach task, on Isaac Lab's deployed reach task |
| `piper_lab/actions.py` | The ramped joint position action |
| `piper_lab/agents.py` | RSL-RL's PPO settings (a 256-128-64 MLP) |

The task is after Isaac Lab's deployed UR10e reach task
([isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab),
`isaaclab_tasks/contrib/deploy/reach`).
