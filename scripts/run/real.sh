#!/usr/bin/env bash
# The stack on the real arm over CAN (can_follower, up at 1 Mbit/s: scripts/setup-host.sh can).
# Read-only unless enable:=true is passed; any motion needs the operator at the arm.
#
#   ./scripts/run/real.sh <variant> [name:=value...] [--print]
#
#   arm      joint states, the controllers and RViz; enable:=true to command the arm
#   camera   arm as above, plus the wrist camera with its frames from the robot description
#   stop     end whichever stack is running
#
# name:=value arguments go to the bringup launch. --print lists the commands instead of opening them.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=can motion:=true rviz:=true"

case "${VARIANT}" in
    arm)
        container bringup "${BRINGUP}${LAUNCH_ARGS}"
        container monitor "ros2 run piper_driver piper_monitor \${PIPER_CAN:-can_follower}"
        ;;
    camera)
        container bringup "${BRINGUP}${LAUNCH_ARGS}"
        container camera "ros2 launch piper_bringup camera.launch.py publish_tf:=false"
        container monitor "ros2 run piper_driver piper_monitor \${PIPER_CAN:-can_follower}"
        ;;
    *) usage ;;
esac
open_panes "Real arm: ${VARIANT}"
