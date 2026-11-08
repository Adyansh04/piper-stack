#!/usr/bin/env bash
# The stack against the Piper in Isaac Sim (sim/sim.py), on the simulator's clock.
#
#   ./scripts/run/sim.sh <variant> [name:=value...] [--print]
#
#   arm      the simulator, then robot_state_publisher and the controllers on hardware:=isaac, RViz
#   moveit   arm, plus MoveIt (OMPL, Pilz, pick_ik) with its RViz panel
#   jog      moveit, plus MoveIt Servo and the keyboard jog in its own pane
#   stop     end the stack (the simulator stops with its pane's Ctrl-C)
#
# The simulator runs headless in the sim container (docker compose --profile sim); its first start
# compiles shaders for minutes. HEADLESS=false opens Isaac Sim's window, PHYSICS_ENGINE=newton
# tries Newton. name:=value arguments go to the bringup launch. --print lists the commands.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=isaac"
# shellcheck disable=SC2016 # expanded in the keyboard pane, inside the container
readonly JOG='ros2 run piper_teleop keyboard_jog --ros-args --params-file "$(ros2 pkg prefix piper_teleop)/share/piper_teleop/config/keyboard_jog.yaml"'
# The robot description the simulator imports, written by the dev container (it has xacro).
readonly URDF='mkdir -p /piper-stack/data/sim && xacro /piper-stack/ros_ws/src/piper_description/urdf/piper.urdf.xacro > /piper-stack/data/sim/piper.urdf'
readonly SIM="./scripts/manage.sh exec bash -c '${URDF}' && HOST_UID=\$(id -u) HOST_GID=\$(id -g) docker compose --profile sim up -d sim && docker exec -it -e HEADLESS=\${HEADLESS:-true} -e PHYSICS_ENGINE=\${PHYSICS_ENGINE:-physx} piper-sim /isaac-sim/python.sh /piper-stack/sim/sim.py"
readonly WAIT_CLOCK="wait_for /clock 'timeout 10 ros2 topic echo --once /clock'"

host simulator "${SIM}"
case "${VARIANT}" in
    arm)
        container bringup "${WAIT_CLOCK} && ${BRINGUP} motion:=true rviz:=true${LAUNCH_ARGS}"
        container controllers "wait_for_controller arm_controller && ros2 control list_controllers"
        ;;
    moveit) container bringup "${WAIT_CLOCK} && ${BRINGUP} motion:=true moveit:=true rviz:=true${LAUNCH_ARGS}" ;;
    jog)
        container bringup "${WAIT_CLOCK} && ${BRINGUP} motion:=true moveit:=true servo:=true rviz:=true${LAUNCH_ARGS}"
        container keyboard "wait_for_service /servo_node/switch_command_type && ${JOG}"
        ;;
    *) usage ;;
esac
open_panes "Sim: ${VARIANT}"
