# piper_skills

The Piper's manipulation skills on one table: move to a named pose, look at the workspace, find
objects by name, pick one and place it. Each skill is an action; a behaviour tree or a person
calls them one at a time.

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

One skill runs at a time: a goal sent while one runs is rejected. A cancel stops the arm's move
and ends the skill between its steps. Picks and places report their phase (`finding`,
`planning_grasps`, `moving_above`, `descending`, `closing`, `lifting`, `releasing`, ...) as
feedback.

Topics in: the wrist camera's `/wrist_camera/color/image_raw`, `/wrist_camera/depth/image_raw`
(16UC1 millimetres or 32FC1 metres, aligned to colour) and `/wrist_camera/color/camera_info`, and
TF. Calls: `segment_objects`, `compute_grasps`, MoveIt's `compute_ik` and `move_group`, and the
gripper controller's `ParallelGripperCommand` action.

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

## Tests

```bash
./scripts/manage.sh test piper_skills
```

`test_skills_core` covers the geometry without ROS: grasp ranking, object clouds, free spots, place
targets, view planning and SRDF states.
