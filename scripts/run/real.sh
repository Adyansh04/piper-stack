#!/usr/bin/env bash
# The stack on the real arm over CAN (can_follower, up at 1 Mbit/s: scripts/setup-host.sh can).
# Read-only unless enable:=true is passed; any motion needs the operator at the arm.
#
#   ./scripts/run/real.sh <variant> [name:=value...] [--print]
#
#   arm      joint states, the controllers and RViz; enable:=true to command the arm
#   camera   arm as above, plus the wrist camera with its frames from the robot description
#   mission  camera, plus MoveIt, SAM 3.1, GraspGenX, the skills and the behaviour tree executor, and
#            a pane that sends a PickAndPlace run each time Enter is pressed (needs enable:=true)
#   stop     end whichever stack is running
#
# name:=value arguments go to the bringup launch. --print lists the commands instead of opening them.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=can motion:=true rviz:=true"
readonly CAMERA="ros2 launch piper_bringup camera.launch.py publish_tf:=false"
readonly MONITOR="ros2 run piper_driver piper_monitor \${PIPER_CAN:-can_follower}"
readonly WAIT_RUN_TREE="wait_for /run_tree 'timeout 10 ros2 action list | grep -qx /run_tree'"
readonly RUN="ros2 action send_goal --feedback /run_tree piper_msgs/action/RunTree \"{tree: PickAndPlace, blackboard: ['object=red cube', 'place_region=left']}\""

case "${VARIANT}" in
    arm)
        container bringup "${BRINGUP}${LAUNCH_ARGS}"
        container monitor "ros2 run piper_driver piper_monitor \${PIPER_CAN:-can_follower}"
        ;;
    camera)
        container bringup "${BRINGUP}${LAUNCH_ARGS}"
        container camera "${CAMERA}"
        container monitor "${MONITOR}"
        ;;
    mission)
        container bringup "${BRINGUP} moveit:=true${LAUNCH_ARGS}"
        container camera "${CAMERA}"
        container monitor "${MONITOR}"
        container segmentation "ros2 launch piper_perception sam3_segmentation.launch.py"
        container grasps "ros2 launch piper_perception grasp_generation.launch.py"
        container skills "wait_for_service /compute_ik && ros2 launch piper_skills skills.launch.py"
        container executor "ros2 launch piper_bt bt_executor.launch.py"
        staged run "${WAIT_RUN_TREE}" "${RUN}"
        ;;
    *) usage ;;
esac
open_panes "Real arm: ${VARIANT}"
