#!/usr/bin/env bash
# The stack against the Piper in Isaac Sim (sim/sim.py), on the simulator's clock.
#
#   ./scripts/run/sim.sh <variant> [name:=value...] [--print]
#
#   arm      the simulator, then robot_state_publisher and the controllers on hardware:=isaac, RViz
#   moveit   arm, plus MoveIt (OMPL, Pilz, pick_ik) with its RViz panel
#   jog      moveit, plus MoveIt Servo and the keyboard jog in its own pane
#   mission  moveit, plus SAM 3.1, GraspGenX, the skills and the behaviour tree executor, and a pane
#            that sends a PickAndPlace run (red cube to the left third) each time Enter is pressed
#   shadow   the simulator alone, as a digital twin: its arm follows /joint_states of the stack that
#            ./scripts/run/real.sh (or mock.sh) runs, its own state on /shadow/joint_states and its
#            camera under /shadow/wrist_camera, with an image view of the two cameras
#   stop     end the stack (the simulator stops with its pane's Ctrl-C)
#
# The simulator runs headless in the sim container (docker compose --profile sim); its first start
# compiles shaders for minutes. HEADLESS=false opens Isaac Sim's window, PHYSICS_ENGINE=newton
# tries Newton, LIVESTREAM=true streams Isaac Sim's UI over WebRTC (for a simulator on another
# machine: NVIDIA's Isaac Sim WebRTC Streaming Client, 49100/tcp and 47998/udp). name:=value
# arguments go to the bringup launch. --print lists the commands.
# shellcheck source=../lib/panes.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

readonly BRINGUP="ros2 launch piper_bringup bringup.launch.py hardware:=isaac"
# shellcheck disable=SC2016 # expanded in the keyboard pane, inside the container
readonly JOG='ros2 run piper_teleop keyboard_jog --ros-args --params-file "$(ros2 pkg prefix piper_teleop)/share/piper_teleop/config/keyboard_jog.yaml"'
# The robot description the simulator imports, written by the dev container (it has xacro).
readonly URDF='mkdir -p /piper-stack/data/sim && xacro /piper-stack/ros_ws/src/piper_description/urdf/piper.urdf.xacro > /piper-stack/data/sim/piper.urdf'
readonly SIM="./scripts/manage.sh exec bash -c '${URDF}' && HOST_UID=\$(id -u) HOST_GID=\$(id -g) docker compose --profile sim up -d sim && docker exec -it -e HEADLESS=\${HEADLESS:-true} -e PHYSICS_ENGINE=\${PHYSICS_ENGINE:-physx} -e SIM_MODE=\${SIM_MODE:-control} -e LIVESTREAM=\${LIVESTREAM:-false} piper-sim /isaac-sim/python.sh /piper-stack/sim/sim.py"
readonly WAIT_CLOCK="wait_for /clock 'timeout 10 ros2 topic echo --once /clock'"
readonly SEGMENT="ros2 launch piper_perception sam3_segmentation.launch.py use_sim_time:=true"
readonly GRASPS="ros2 launch piper_perception grasp_generation.launch.py"
readonly SKILLS="ros2 launch piper_skills skills.launch.py use_sim_time:=true"
readonly EXECUTOR="ros2 launch piper_bt bt_executor.launch.py use_sim_time:=true"
readonly WAIT_RUN_TREE="wait_for /run_tree 'timeout 10 ros2 action list | grep -qx /run_tree'"
readonly RUN="ros2 action send_goal --feedback /run_tree piper_msgs/action/RunTree \"{tree: PickAndPlace, blackboard: ['object=red cube', 'place_region=left']}\""

if [[ "${VARIANT}" == shadow ]]; then
    host simulator "export SIM_MODE=shadow && ${SIM}"
else
    host simulator "${SIM}"
fi
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
    mission)
        container bringup "${WAIT_CLOCK} && ${BRINGUP} motion:=true moveit:=true rviz:=true${LAUNCH_ARGS}"
        container segmentation "${WAIT_CLOCK} && ${SEGMENT}"
        container grasps "${GRASPS}"
        container skills "wait_for_service /compute_ik && ${SKILLS}"
        container executor "${EXECUTOR}"
        staged run "${WAIT_RUN_TREE}" "${RUN}"
        ;;
    shadow)
        container cameras "wait_for /shadow/joint_states 'timeout 10 ros2 topic echo --once /shadow/joint_states' && ros2 run rqt_image_view rqt_image_view /shadow/wrist_camera/color/image_raw"
        ;;
    *) usage ;;
esac
open_panes "Sim: ${VARIANT}"
