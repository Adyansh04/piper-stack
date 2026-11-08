# Missions

Picking and placing by behaviour tree: the trees of `piper_bt` over the skills of `piper_skills`,
in Isaac Sim or on the real arm.

## Start the stack

```bash
./scripts/run/sim.sh mission
./scripts/run/real.sh mission enable:=true
```

Both open the stack with MoveIt, SAM 3.1 (`segment_objects`), GraspGenX (`compute_grasps`), the
skills and the executor, plus a pane that sends one `PickAndPlace` run each time Enter is pressed.
SAM 3.1 builds its TensorRT engines on the first start (a few minutes); GraspGenX loads in about
40 s.

## Run a tree

From any container shell:

```bash
ros2 action send_goal --feedback /run_tree piper_msgs/action/RunTree \
  "{tree: PickAndPlace, blackboard: ['object=red cube', 'place_region=left']}"
ros2 action send_goal --feedback /run_tree piper_msgs/action/RunTree \
  "{tree: SortObjects, blackboard: ['object_1=red cube', 'region_1=left', 'object_2=blue cube', 'region_2=right']}"
ros2 action send_goal --feedback /run_tree piper_msgs/action/RunTree \
  "{tree: ClearWorkspace, blackboard: ['object=cube', 'region=right']}"
```

The feedback lists the leaves running; the result says SUCCESS, FAILURE (with the failing leaf's
reason) or HALTED. Each tree's keys are in its file's `TreeNodesModel` and in `piper_bt`'s README;
a run with an unknown key, or without a required one, is refused.

## Control a run

```bash
ros2 service call /bt_executor/control piper_msgs/srv/ControlTree "{command: 1}"
```

`1` pauses (the running skill finishes its step), `2` holds (the running skill stops), `3`
resumes. Cancelling the `run_tree` goal (Ctrl-C on `send_goal`) halts the tree and stops the arm.

## Watch it

- Groot2: start the executor with `groot2_port` 1667 (see `piper_bt`'s README) and connect.
- `ros2 topic echo /bt_executor/status`: the tree, each node's status and the blackboard.
- `ros2 launch piper_bringup record.launch.py profile:=mission`: a bag of the run.

## Limits

The skills grasp and place with the gripper pointing down. The Piper's wrist allows that only low
over the table: objects and place spots within about 0.33 m of the arm's base work, farther ones
fail with a reason (no reachable grasp, no free spot within reach).
