# Architecture

How piper-stack is put together: the packages, how data flows between them, and how one code path
serves the mock arm, Isaac Sim and the real arm.

## Contents

1. Layers
2. One code path for sim and real
3. A pick and place, step by step
4. Images and machines
5. Decisions

## 1. Layers

From the motors up:

| Layer | Packages | What it does |
|---|---|---|
| CAN | `piper_driver` | The Piper's protocol V2 over SocketCAN, without ROS |
| Control | `piper_hardware`, `piper_controllers`, `piper_bringup` | `ros2_control`: the hardware system, a joint trajectory controller, a gripper controller and a trained reach policy (ONNX) at 200 Hz |
| Description | `piper_description`, `piper_moveit_config` | The URDF with the wrist camera, MoveIt's groups, pick_ik, OMPL, Pilz, Servo |
| Perception | `piper_perception` | SAM 3.1 segmentation (TensorRT, C++) and GraspGenX grasps (a Python server behind a C++ service) |
| Skills | `piper_skills` | Named poses, views, find, pick, place, place target: one action or service each |
| Missions | `piper_bt` | Behaviour trees over the skills, run through `run_tree`; Groot2 edits and watches them |
| Teleop and estimates | `piper_teleop`, `piper_estimation` | Keyboard jogging through Servo; the wrench at the tool and grasp detection from joint torques |
| Data and policies | `piper_lerobot` | LeRobot's robot and teleoperator over a ZMQ bridge node: recording datasets, policies' joint targets through Servo |
| Interfaces | `piper_msgs` | The stack's messages, services and actions |

Each layer only calls the one below through ROS interfaces: the skills call MoveIt and the
perception services, the trees call the skills, nothing above `ros2_control` writes to the CAN bus.

## 2. One code path for sim and real

The bringup's `hardware` argument picks the `ros2_control` system in the robot description, and
nothing else changes:

| `hardware` | System | Clock |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | Wall |
| `isaac` | ros-controls' `JointStateTopicSystem` on `/isaac_joint_commands` and `/isaac_joint_states` | Isaac Sim's `/clock` |
| `can` | `piper_hardware/PiperSystem` over `can_follower` | Wall |

Isaac Sim (`sim/`, Python, in its own image) closes the loop in its physics callback at 200 Hz: it
takes the controllers' position targets, steps PhysX and publishes the joint states and `/clock`.
Its RTX wrist camera publishes the topics Percipio's driver publishes on the real camera, in the
same frames, at the real camera's default resolution and focal length, so the perception and the
skills see no difference. In shadow mode the
simulator is a digital twin instead: it follows the real arm's `/joint_states`.

Isaac Lab (`lab/`) trains on the same robot USD, drives and table, and with the policy
controller's own motion law and parameters (`controllers.yaml`), so a policy exported to ONNX runs
unchanged in `piper_controllers`' policy controller on any of the three systems.

## 3. A pick and place, step by step

`run_tree PickAndPlace` with `object=red cube` and `place_region=left`:

1. The executor ticks `PickFromTable`; its `PickObject` leaf sends a `pick_object` goal.
2. The skills move to a view (an SRDF state), take a colour and depth pair from the wrist camera
   and ask `segment_objects` (SAM 3.1) for "red cube".
3. The best mask becomes the object's points from the depth image; `compute_grasps` (GraspGenX)
   returns grasps, which the skills rank (pads, opening, depth, the table) and check with
   `compute_ik`.
4. MoveIt moves to the pre-grasp pose (Pilz PTP, OMPL as fallback), goes in along the approach
   (Pilz LIN), the gripper closes, the fingers' travel is checked against the object's width, and
   the arm lifts with the object attached to the TCP in MoveIt's planning scene.
5. `PlaceOnTable`'s `PlaceObject` leaf sends `place_object`: the skills look at the region (and
   from the other views when the held object hides it), find a free spot within the arm's reach in
   the depth image, and put the object down there.
6. A failure on the way is retried by the tree (look again, try again, back to `ready`), and the
   reason ends up in the run's result.

## 4. Images and machines

- `piper-dev` (`docker/dev/Dockerfile`): ROS 2 Lyrical on Ubuntu 26.04 with everything the stack
  runs: MoveIt, CUDA and TensorRT, the GraspGenX Python 3.12 environment, BehaviorTree.CPP,
  PlotJuggler 4. The checkout is mounted, never copied in. A CI workflow publishes it to GHCR
  under a hash of its inputs.
- `piper-sim` (`docker/sim/Dockerfile`): Isaac Sim 6.2 with Isaac Lab 3.0. Its ROS 2 bridge speaks
  Jazzy; the two containers meet over Fast DDS on the host network.
- Everything runs on one PC by default, and always on the real arm. Isaac Sim can also run on a GPU
  server with its UI streamed over WebRTC (`LIVESTREAM=true`), the stack beside it there.

## 5. Decisions

- C++ for everything at or above 50 Hz, the skills, the trees and perception's hot path; Python
  only for Isaac Sim, Isaac Lab, the GraspGenX server and launch files.
- The real arm's firmware interpolates every target; the controllers stream positions and the
  firmware's speed scaling applies (MIT mode bypasses it, with per-joint gains).
- The simulator is a `ros2_control` system over topics rather than a controller manager inside
  Isaac Sim, so the controllers and MoveIt are the same in sim and on the arm.
- The skills plan with the gripper pointing down, which the Piper's wrist (joint 5 within 70 deg)
  reaches only low over the table: places stay within 0.33 m of the base, and approaches are 6 cm.
