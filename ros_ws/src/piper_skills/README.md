# piper_skills

The Piper's manipulation skills on one table: move to a named pose, look at the workspace, find
objects by name, pick one and place it. Each skill is an action; a behaviour tree or a person
calls them one at a time. `task_scene` keeps a contact task's parts in MoveIt's planning scene.

## Run

With the stack (`piper_bringup` with `moveit:=true`), `segment_objects` and `compute_grasps`
(`piper_perception`) running:

```bash
ros2 launch piper_skills skills.launch.py
ros2 action send_goal /pick_object piper_msgs/action/PickObject "{prompt: red cube}"
ros2 action send_goal /place_object piper_msgs/action/PlaceObject "{mode: free, region: left}"
```

Against Isaac Sim, add `use_sim_time:=true` (`./scripts/run/sim.sh moveit` starts the simulator and
the stack).

## Interfaces

| Name | Type | What it does |
|---|---|---|
| `move_arm_to_named_pose` | `piper_msgs/action/MoveArmToNamedPose` | Moves to an SRDF state (`rest`, `ready`, `view_left`, `view_right`) |
| `view_workspace` | `piper_msgs/action/ViewWorkspace` | Points the wrist camera at `full`, `left`, `middle` or `right` |
| `find_objects` | `piper_msgs/action/FindObjects` | Segments a prompt in each view in turn; returns the first view's objects (pose, size, score) |
| `pick_object` | `piper_msgs/action/PickObject` | Finds the best match of a prompt, asks GraspGenX for grasps, ranks and tries them |
| `place_object` | `piper_msgs/action/PlaceObject` | Puts the held object on a free spot: in a region (`free`), at `relative` (-1 to 1) or `metric` coordinates, or next to an `anchor` object |
| `place_target` | `piper_msgs/srv/PlaceTarget` | Where a place goal's target lies, and whether the arm reaches above it |
| `run_stage` | `piper_msgs/action/RunStage` | Runs one stage of a contact task (`task.file`): grasps its part, puts it on its mate, screws or twists it home (below) |

One skill runs at a time: a goal sent while one runs is rejected. A cancel stops the arm's move
and ends the skill between its steps. Picks and places report their phase (`finding`,
`planning_grasps`, `moving_above`, `descending`, `closing`, `lifting`, `releasing`, ...) as
feedback.

Topics in: the wrist camera's `/wrist_camera/color/image_raw`, `/wrist_camera/depth/image_raw`
(16UC1 millimetres or 32FC1 metres, aligned to colour) and `/wrist_camera/color/camera_info`, and
TF. Calls: `segment_objects`, `compute_grasps`, MoveIt's `compute_ik` and `move_group`, and the
gripper controller's `ParallelGripperCommand` action.

## Task scene

`task_scene` keeps MoveIt's planning scene in step with a task's parts (`tasks/<task>/task.yaml`),
so planning and MoveIt Servo know where the fixture and every part are. Each part with collision
shapes becomes a collision object of its meshes. Its pose comes from `/sim/objects`
(`tf2_msgs/TFMessage`, child frame = the part's name, in `world`): Isaac Sim's ground truth, or a
publisher of the fixture's calibrated pose on the real arm. A part that moves more than 0.1 mm
or 0.002 rad is moved in the scene within `update_period_s` (0.1 s).

```bash
ros2 launch piper_skills task_scene.launch.py task_file:=/piper-stack/tasks/assembly/task.yaml \
    use_sim_time:=true
ros2 service call /task_scene/attach piper_msgs/srv/AttachPart "{part: ring, attach: true}"
ros2 service call /task_scene/allow_contacts piper_msgs/srv/AllowContacts \
    "{first: [collar], second: [base], allowed: true}"
```

| Service | Type | What it does |
|---|---|---|
| `~/attach` | `piper_msgs/srv/AttachPart` | Attaches a part to `tcp_link` (it may touch `touch_links`), or detaches it back into the world |
| `~/allow_contacts` | `piper_msgs/srv/AllowContacts` | Allows or forbids contacts between pairs of parts or links (`first[i]` with `second[i]`) in the allowed collision matrix |

A stage that mates two parts allows their contact for its duration: MoveIt Servo halts the arm
before a contact the matrix forbids. `bringup.launch.py task_file:=...` starts the node with the
stack; its parameters are in `config/task_scene.yaml`.

## Task stages

Started with the same task (`skills.launch.py task_file:=...`), the skills run the stages of its
`task.yaml` (see [tasks/README.md](../../../tasks/README.md)) through `run_stage`:

```bash
ros2 action send_goal --feedback /run_stage piper_msgs/action/RunStage "{stage: ring}"
```

A stage takes its part where MoveIt's scene has it and grasps it with the grasp (of the part's
symmetry) that the arm reaches both there and above the part's mate. It lifts the part (attached
in the scene) and looks where it sits in the fingers: let go of in the scene for
`task.scene_settle_s`, the part shows where it was last measured. It then brings the part, not the
grasp, above its mate and down in a straight line (Pilz LIN at `task.insert_velocity_scale`),
then:

- `insert`: lets go where the goal is. With a `guide` (`tasks/README.md`) the part is first
  aligned out of contact from where the scene sees it, then goes its last millimetres in moves
  pressed a little further; where a move gains no depth it backs out, aligns again and comes in at
  the next point of a spiral round the axis.
- `screw`: turns the part `task.screw_step_deg` a move from where the scene sees it, pressed
  `task.screw_preload_m` below the height its thread gives it, so the thread sets how far it goes
  down, until it is within `task.screw_seated_m` of its seat. When `joint6` comes within
  `task.wrist_margin_rad` of its limit the fingers let go, the wrist turns back half a turn and
  the fingers close on the part's flats again at the grasp's height on it. A thread that has not
  taken the part after its turns and `task.screw_search_turns` more fails the stage.
- `twist`: turns the part in place by the stage's twist, in moves of at most
  `task.max_turn_step_deg`.

The fingers then open, the arm rises and the part's pose in the scene is checked against its
goal (`task.goal_tolerance_m`, `task.goal_tolerance_deg`). The fingers may touch the part, and the
stage's contacts are allowed, only while the stage runs. The part's goal and the path to it come
from the task, its pose and its mate's from the scene: ground truth in the simulator.

## How a pick works

1. Look at the table from the first view that shows the prompt; turn each mask into the object's
   points from the depth image, keeping only what stands on the table.
2. Send the points to `compute_grasps`; rank the grasps by score, uprightness, depth over the
   object, opening and contact spread, within the pads' and the table's limits.
3. For the best grasps (each also turned half a turn about its approach): move above, go down in
   a straight line (Pilz LIN), close, check that the fingers stopped at the object's width, lift.
4. Attach the object to the TCP in MoveIt's planning scene and return to `ready`.

A place looks at the region from its own view, then from the other views when the gripper and the
held object hide the free table under the camera. It takes the free spot (no object within
`free_radius_m` or the object's own radius, at most `max_reach_m` from the arm's base) nearest the
region's centre and the robot, and puts the object down there with the attitude it was picked in,
turned as needed.

## Parameters

All in `config/skills.yaml`, with units and reasons. The ones to know:

- `table.*`: the table's top in the planning frame; must agree with the simulator's
  `sim/config/sim.yaml`.
- `gripper.*`: the finger travel (`open_m` 0.035 = 70 mm between the fingers), effort and the
  held-object checks.
- `grasp.*`: the pads' size and the grasp limits and ranking weights.
- `workspace.*`: the part of the table the skills find on and place on; its thirds across are
  the regions `left`, `middle` and `right`.
- `view.*`: the camera's fields of view and the views, SRDF states that show the workspace.
- `pick.*`, `place.*`: approach and lift heights, and `place.max_reach_m`, beyond which the arm
  cannot come down from above (with the gripper straight down it reaches 0.08 to 0.12 m above the
  top within 0.33 m of its base).
- `task.*`: the contact task (`task.file`), the speeds of its insertions and turns, the wrist's
  margin before a screw regrasps, and how far a stage's part may end from its goal.

## Tests

```bash
./scripts/manage.sh test piper_skills
```

`test_skills_core` covers the geometry without ROS: grasp ranking, object clouds, free spots, place
targets, view planning, SRDF states and a task's stages (parsing, grasps, the screw's and twist's
paths).
