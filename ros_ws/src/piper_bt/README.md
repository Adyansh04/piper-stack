# piper_bt

The Piper's behaviour tree executor ([BehaviorTree.CPP](https://www.behaviortree.dev/) 4). It runs
one tree at a time through the `run_tree` action: missions such as "pick the red cube and put it in
the left third", built from one leaf per skill of `piper_skills`, with retries and a way back to
`ready` when a step fails.

## Run

With the stack, perception and the skills up (`piper_bringup` with `moveit:=true`,
`piper_perception`, `piper_skills`):

```bash
ros2 launch piper_bt bt_executor.launch.py
ros2 action send_goal /run_tree piper_msgs/action/RunTree \
  "{tree: PickAndPlace, blackboard: ['object=red cube', 'place_region=left']}" --feedback
```

Against Isaac Sim, add `use_sim_time:=true`.

## Trees

Missions (`trees/missions`), each with its keys in the file's `TreeNodesModel`:

| Tree | Keys | What it does |
|---|---|---|
| `PickAndPlace` | `object`, `place_mode`, `place_region`, `place_x`, `place_y`, `place_anchor`, ... | Picks an object and puts it on the table, then back to `ready` |
| `SortObjects` | `object_1` to `object_3`, `region_1` to `region_3` | Up to three moves in order; an object left out (`none`) is skipped |
| `ArrangeInRow` | `objects` (`;`-separated), `spacing_m`, `row_y_m` | A row from left to right, `spacing_m` apart |
| `ClearWorkspace` | `object`, `region` | Moves every match to a region until a look finds none |
| `ArmToPose` | `pose` | Moves the arm to an SRDF state |
| `AssembleStages` | `stages` (`;`-separated) | Runs a contact task's stages in order; the first that fails ends the run back at `ready` |

Subtrees (`trees/library`): `PickFromTable` (three tries, looking again after each failed one) and
`PlaceOnTable` (two tries). Both end with the arm back at `ready` when every try fails. A skill
that reports the arm stuck (the fingers would not open or let go) stops every move after it.

The executor refuses a run whose keys are not in the tree's model, or that leaves out a key without
a default.

## Leaves

| Leaf | Calls |
|---|---|
| `MoveArmToNamedPose` | `move_arm_to_named_pose` |
| `ViewWorkspace` | `view_workspace` |
| `FindObjects` | `find_objects` |
| `PickObject` | `pick_object` |
| `PlaceObject` | `place_object` |
| `PlaceTarget` | `place_target` |
| `RunStage` | `run_stage` |

Every leaf keeps a tick short: it sends its goal, stays RUNNING, and fails with the reason in the
root blackboard's `failure` entry when the server is missing, refuses, aborts or takes longer than
its time limit. The ports and their documentation are in `groot/piper_bt_nodes.xml`, the palette
for [Groot2](https://www.behaviortree.dev/groot/), generated with
`ros2 run piper_bt bt_node_model`.

## Groot2

To edit trees, load `groot/piper_bt_nodes.xml` in Groot2 as the palette and open the files in
`trees/`. To watch a run live, start the executor with `groot2_port` 1667 and connect Groot2's
monitor to that host and port:

```bash
ros2 run piper_bt bt_executor --ros-args \
  --params-file "$(ros2 pkg prefix piper_bt)/share/piper_bt/config/bt_executor.yaml" \
  -p groot2_port:=1667
```

## Interfaces

| Name | Type | What it does |
|---|---|---|
| `run_tree` | `piper_msgs/action/RunTree` | Runs a tree to its end; feedback lists the running leaves |
| `~/control` | `piper_msgs/srv/ControlTree` | Pause (running skills finish their step), hold (they stop), resume |
| `~/status` | `piper_msgs/msg/TreeStatus` | The running tree's XML, node statuses and blackboard (transient local) |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | The run in one line for monitors |

A cancel of `run_tree` halts the tree; the running skill's goal is cancelled and the arm stops.

## Parameters

All in `config/bt_executor.yaml`. The ones to know:

- `tree_directories`: where the trees load from (`package://piper_bt/trees`).
- `tick_rate_hz`: 20 ticks per second.
- `groot2_port`: 0 (off); 1667 lets Groot2 watch a run.
- `server_timeout_s`, `goal_timeout_s`, `short_goal_timeout_s`: the leaves' time limits where a
  tree sets none.

## Tests

```bash
./scripts/manage.sh test piper_bt
```

`test_tree_loads` builds every shipped tree and runs the missions on scripted stand-ins for the
leaves (retries, the way back, a stuck arm, the row's spacing, clearing until none is left).
`test_leaves` runs the leaves against in-process action and service servers, and `run_tree` with
pause, hold, resume and cancel.
