#!/usr/bin/env bash
# The stack on ros2_control's mock hardware: no arm, no simulator.
#
#   ./scripts/run/mock.sh <variant> [name:=value...] [--print]
#
#   arm    robot_state_publisher, the controllers and RViz; the controllers follow every command
#   stop   end whichever stack is running
#
# name:=value arguments go to the bringup launch. --print lists the commands instead of opening them.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=mock"

case "${VARIANT}" in
    arm)
        container bringup "${BRINGUP} motion:=true rviz:=true${LAUNCH_ARGS}"
        container controllers "wait_for_controller arm_controller && ros2 control list_controllers"
        ;;
    *) usage ;;
esac
open_panes "Mock: ${VARIANT}"
