# piper_msgs

The stack's own interfaces: perception, the pick and place skills, the contact tasks' planning
scene and the behaviour tree executor. Poses are in the planning frame (`world`, the robot's base at the table top) unless a
field says otherwise.

## Messages

| Message | What it holds |
|---|---|
| `DetectedObject` | An object found on the table: label, score, pose, size and its points' count |
| `Segment` | One instance mask from the open-vocabulary segmenter, with its box and score |
| `Grasp` | A parallel-gripper grasp: TCP pose, opening, the object's width there, score |
| `BlackboardEntry` | One entry of a behaviour tree's blackboard |
| `TreeNodeStatus` | One node of the running tree and its status |
| `TreeStatus` | The executor's state: the tree, its nodes' statuses and its blackboard |

## Services

| Service | Served by | What it does |
|---|---|---|
| `SegmentObjects` | `piper_perception` (`sam3_segmentation`) | Instance masks of an image for text prompts |
| `ComputeGrasps` | `piper_perception` (`grasp_generation`) | Ranked grasps for an object's points |
| `PlaceTarget` | `piper_skills` | Where a place target lies on the table, and whether the arm reaches it |
| `AttachPart` | `piper_skills` (`task_scene`) | Attaches a task part to the gripper in MoveIt's scene, or detaches it |
| `AllowContacts` | `piper_skills` (`task_scene`) | Allows or forbids contacts between pairs of parts or links in MoveIt's scene |
| `ControlTree` | `piper_bt` | Pause, hold or resume the running tree |

## Actions

| Action | Served by | What it does |
|---|---|---|
| `MoveArmToNamedPose` | `piper_skills` | Moves the arm to an SRDF state such as `ready` |
| `ViewWorkspace` | `piper_skills` | Points the wrist camera at the work area or one of its thirds |
| `FindObjects` | `piper_skills` | Looks for a prompt view by view |
| `PickObject` | `piper_skills` | Finds, grasps and lifts an object |
| `PlaceObject` | `piper_skills` | Puts the held object on a free spot or at a target |
| `RunTree` | `piper_bt` | Runs one behaviour tree to its end |

Each file's comments give the fields' units and meanings.

## Tests

None of its own: the packages that serve and call the interfaces test them.
