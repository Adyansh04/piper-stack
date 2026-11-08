#!/usr/bin/env bash
# The stack on ros2_control's mock hardware: no arm, no simulator.
#
#   ./scripts/run/mock.sh <variant> [name:=value...] [--print]
#
#   arm      robot_state_publisher, the controllers and RViz; the controllers follow every command
#   moveit   arm, plus MoveIt (OMPL, Pilz, pick_ik) with its RViz panel
#   jog      moveit, plus MoveIt Servo and the keyboard jog in its own pane
#   stop     end whichever stack is running
#
# name:=value arguments go to the bringup launch. --print lists the commands instead of opening them.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=mock"
# shellcheck disable=SC2016 # expanded in the keyboard pane, inside the container
readonly JOG='ros2 run piper_teleop keyboard_jog --ros-args --params-file "$(ros2 pkg prefix piper_teleop)/share/piper_teleop/config/keyboard_jog.yaml"'

case "${VARIANT}" in
    arm)
        container bringup "${BRINGUP} motion:=true rviz:=true${LAUNCH_ARGS}"
        container controllers "wait_for_controller arm_controller && ros2 control list_controllers"
        ;;
    moveit) container bringup "${BRINGUP} motion:=true moveit:=true rviz:=true${LAUNCH_ARGS}" ;;
    jog)
        container bringup "${BRINGUP} motion:=true moveit:=true servo:=true rviz:=true${LAUNCH_ARGS}"
        container keyboard "wait_for_service /servo_node/switch_command_type && ${JOG}"
        ;;
    *) usage ;;
esac
open_panes "Mock: ${VARIANT}"
