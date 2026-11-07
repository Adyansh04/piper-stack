#!/usr/bin/env bash
# Runs the piper-stack dev container and the everyday commands inside it.
#
#   ./scripts/manage.sh <command> [args]     (help lists the commands)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROOT
readonly SERVICE=dev
cd "${ROOT}"

if [[ -f .env ]]; then
    set -a
    # shellcheck disable=SC1091
    source .env
    set +a
fi
# The image gives its user the host's IDs, so files written in the checkout stay the user's.
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
export HOST_UID HOST_GID

# Our packages: the vendored ones are neither tested nor linted here.
readonly OUR_PACKAGES='^piper_'

# Before every command in the container: ROS, then the workspace once it is built.
readonly IN_WORKSPACE='source /opt/ros/lyrical/setup.bash && cd /piper-stack/ros_ws && if [[ -f install/setup.bash ]]; then source install/setup.bash; fi'

usage() {
    cat <<EOF
Usage: $0 <command> [args]
  start             Start the dev container (builds the image on first use).
  stop              Stop the container.
  recreate          Rebuild the image if its inputs changed, and start a new container.
  logs              Follow the container's output.
  exec [cmd...]     A shell in ros_ws with the workspace sourced, or run cmd there.
  build [pkg...]    colcon build the named packages and their dependencies, or everything.
  test [pkg...]     Our packages' tests (or the named packages').
EOF
}

in_container() {
    local tty=()
    [[ -t 0 && -t 1 ]] || tty=(-T)
    docker compose exec "${tty[@]}" "${SERVICE}" bash -c "${IN_WORKSPACE} && exec \"\$@\"" bash "$@"
}

# Capped: uncapped, the compilers run the laptop out of memory.
build() {
    local select=()
    (($#)) && select=(--packages-up-to "$@")
    in_container env MAKEFLAGS="${MAKEFLAGS:--j6}" colcon build --symlink-install \
        --parallel-workers 4 "${select[@]}" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DCMAKE_LINKER_TYPE=MOLD
}

run_tests() {
    local select=(--packages-select-regex "${OUR_PACKAGES}")
    (($#)) && select=(--packages-select "$@")
    in_container bash -c '
        colcon test "$@" --return-code-on-test-failure --event-handlers console_direct+
        status=$?
        colcon test-result --verbose
        exit "${status}"
    ' bash "${select[@]}"
}

case "${1:-help}" in
    start)
        xhost +local:docker >/dev/null 2>&1 || true
        docker compose up -d --build "${SERVICE}"
        ;;
    stop) docker compose stop "${SERVICE}" ;;
    recreate)
        xhost +local:docker >/dev/null 2>&1 || true
        docker compose up -d --build --force-recreate "${SERVICE}"
        ;;
    logs) docker compose logs -f "${SERVICE}" ;;
    exec)
        shift
        (($#)) || set -- bash
        in_container "$@"
        ;;
    build)
        shift
        build "$@"
        ;;
    test)
        shift
        run_tests "$@"
        ;;
    help | -h | --help) usage ;;
    *)
        usage >&2
        exit 1
        ;;
esac
