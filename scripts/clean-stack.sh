#!/usr/bin/env bash
# Tears the stack down and checks the ROS graph is empty, rather than trusting the process table.
# Run it between launches; it exits non-zero if anything is left, so it can gate a test run.
#
#   ./scripts/clean-stack.sh
#
# Runs from the host or inside the dev container. Leftover nodes cause phantom bugs, such as a
# second controller_manager aborting a launch or two robot_state_publishers fighting over TF.
#
# Matches full command lines: `pgrep -x` reads /proc/PID/comm, truncated to 15 characters, so
# names like ros2_control_node never match. Loops until the graph is empty, since launch respawns
# children and DDS reports dead nodes for seconds. Skips its own ancestry, whose command lines
# contain the same patterns.
# A teardown carries on past failures, so only -u: each step checks for itself.
set -u

readonly CONTAINER=${CONTAINER:-piper-dev}

# From the host, re-run inside the container: a host sweep would ask the host's own ROS for a
# node list and report clean while the container is still full.
if [[ ! -d /piper-stack/ros_ws ]]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "Not in the container and docker is not on PATH." >&2
        exit 1
    fi
    if [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null)" != true ]]; then
        echo "${CONTAINER} is not running: nothing to clean."
        exit 0
    fi
    # Copied to a file, not piped to `bash -s`: the first command that reads stdin would swallow
    # the rest of the script.
    exec docker exec -i "${CONTAINER}" bash -c \
        'f=$(mktemp) && cat > "$f" && bash "$f"; s=$?; rm -f "$f"; exit $s' <"$0"
fi

# NAMED is the stack's own binaries. ANY_ROS catches anything else running from a ROS install or
# with --ros-args, including composed nodes, which never appear by name, and ros2 CLI calls: an
# echo still waiting for a message keeps its hidden node on the graph.
readonly NAMED='ros2_control_node|robot_state_publisher|controller_manager|spawner|move_group|servo_node|rviz2|component_container|piper_monitor|piper_fake_arm'
readonly ANY_ROS='rclcpp_components|--ros-args|/opt/ros/[a-z]+/lib/|ros2 run |ros2 launch |ros2 daemon|bin/ros2 (topic|service|action|param|lifecycle|control|bag|node) '

protected=" $$ ${PPID} "
walk=${PPID}
while [[ -n "${walk}" ]] && ((walk > 1)); do
    walk=$(awk '{print $4}' "/proc/${walk}/stat" 2>/dev/null)
    [[ -n "${walk}" ]] && protected+="${walk} "
done

sweep() {
    local pattern=$1 killed=0 proc pid cmd
    for proc in /proc/[0-9]*; do
        pid=${proc#/proc/}
        [[ "${protected}" == *" ${pid} "* ]] && continue
        # A process may exit mid-sweep; stderr is redirected first so its failure stays quiet.
        cmd=$(tr '\0' ' ' 2>/dev/null <"${proc}/cmdline") || continue
        [[ -z "${cmd}" ]] && continue
        if grep -qE "${pattern}" <<<"${cmd}"; then
            kill -9 "${pid}" 2>/dev/null && killed=$((killed + 1))
        fi
    done
    echo "${killed}"
}

graph_nodes() {
    timeout 25 ros2 node list 2>/dev/null | grep -v '^$'
}

total=0
for pass in 1 2 3 4; do
    n=$(sweep "${NAMED}")
    m=$(sweep "${ANY_ROS}")
    total=$((total + n + m))
    echo "pass ${pass}: killed ${n} by name, ${m} by ROS signature"
    ((n + m == 0 && pass > 1)) && break
    sleep 4
done

# ROS's setup scripts reference unbound variables, which would abort under -u.
set +u
# shellcheck disable=SC1091 # sourced inside the container
source /opt/ros/lyrical/setup.bash
[[ -f /piper-stack/ros_ws/install/setup.bash ]] && source /piper-stack/ros_ws/install/setup.bash
set -u
# The daemon keeps reporting nodes that are gone; restart it so the check below is real.
ros2 daemon stop >/dev/null 2>&1
sleep 3

if [[ -n "$(graph_nodes)" ]]; then
    echo "graph not empty after the sweeps; trying once more"
    total=$((total + $(sweep "${ANY_ROS}")))
    sleep 4
    ros2 daemon stop >/dev/null 2>&1
    sleep 3
fi
echo "killed ${total} process(es) in total, inside the container"

# kill -9 leaves Fast DDS shared-memory segments behind, and stale ones break delivery on later
# runs without an error; `shm clean` removes only those no live process holds (the IPC is the host's).
fastdds shm clean >/dev/null 2>&1 && echo "cleared stale Fast DDS shared-memory segments"

nodes=$(graph_nodes)
topics=$(timeout 25 ros2 topic list 2>/dev/null | grep -vE '^/parameter_events$|^/rosout$')
# report <what is clean> <names, one per line>
report() {
    local name
    if [[ -z "$2" ]]; then
        echo "  ($1)"
        return
    fi
    while IFS= read -r name; do
        echo "  LEFTOVER: ${name}"
    done <<<"$2"
}

echo "--- ros2 node list (want: empty) ---"
report empty "${nodes}"
echo "--- ros2 topic list (want: only /parameter_events and /rosout) ---"
report clean "${topics}"

[[ -z "${nodes}" && -z "${topics}" ]]
