#!/usr/bin/env bash
# The wrist camera on its own (Percipio GM461-E1 over Ethernet), with its colour, depth and cloud
# in RViz. Its frames come from the driver here, not from the robot description.
#
#   ./scripts/run/camera.sh <variant> [name:=value...] [--print]
#
#   view   the driver and RViz
#   stop   end whichever stack is running
#
# name:=value arguments go to the camera launch (device_ip, resolutions, frame_rate).
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

case "${VARIANT}" in
    view) container camera "ros2 launch piper_bringup camera.launch.py rviz:=true${LAUNCH_ARGS}" ;;
    *) usage ;;
esac
open_panes "Camera: ${VARIANT}"
