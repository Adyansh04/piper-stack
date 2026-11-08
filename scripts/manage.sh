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
DEV_IMAGE_TAG="$("${ROOT}/scripts/dev-image-tag.sh")"
export HOST_UID HOST_GID DEV_IMAGE_TAG

# Our packages: the vendored ones are neither tested nor linted here.
readonly OUR_PACKAGES='^piper_'

# Before every command in the container: ROS, then the workspace once it is built.
readonly IN_WORKSPACE='source /opt/ros/lyrical/setup.bash && cd /piper-stack/ros_ws && if [[ -f install/setup.bash ]]; then source install/setup.bash; fi'

usage() {
    cat <<EOF
Usage: $0 <command> [args]
  start             Start the dev container (pulls or builds its image on first use).
  stop              Stop the container.
  recreate          Start a new container on the image for docker/dev as it is now.
  logs              Follow the container's output.
  exec [cmd...]     A shell in ros_ws with the workspace sourced, or run cmd there.
  build [pkg...]    colcon build the named packages and their dependencies, or everything.
  test [pkg...]     Our packages' tests (or the named packages').
  format            clang-format our C++ in place (.clang-format at the repository root).
  lint              clang-tidy on our built packages, ruff and shellcheck; zero findings pass.
EOF
}

# Our C++: the piper_* packages, never the vendored ones.
format_sources() {
    # shellcheck disable=SC2016 # expanded in the container
    in_container bash -c '
        shopt -s globstar nullglob
        files=(/piper-stack/ros_ws/src/piper_*/**/*.{hpp,cpp})
        clang-format --style=file -i "${files[@]}"
        echo "formatted ${#files[@]} files"
    '
}

# The image CI published for this checkout's docker/dev, pulled once; built here when there is
# none yet or when this user's IDs differ from the published image's 1000:1000.
ensure_image() {
    if docker image inspect "ghcr.io/adyansh04/piper-stack-dev:${DEV_IMAGE_TAG}" >/dev/null 2>&1; then
        return
    fi
    if [[ "${HOST_UID}:${HOST_GID}" == 1000:1000 ]] && docker compose pull "${SERVICE}"; then
        return
    fi
    docker compose build "${SERVICE}"
}

# clang-tidy reads each package's compile_commands.json, so it covers what was built.
lint() {
    # shellcheck disable=SC2016 # expanded in the container
    in_container bash -c '
        status=0
        for db in build/piper_*/compile_commands.json; do
            pkg=$(basename "$(dirname "${db}")")
            run-clang-tidy -p "build/${pkg}" -j 4 -quiet "/piper-stack/ros_ws/src/${pkg}/" \
                >"build/${pkg}/clang-tidy.log" 2>&1 || { status=1; grep -E "(warning|error):" "build/${pkg}/clang-tidy.log"; }
        done
        ruff check /piper-stack || status=1
        shellcheck -x -P SCRIPTDIR /piper-stack/scripts/*.sh /piper-stack/scripts/*/*.sh || status=1
        exit "${status}"
    '
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
    # shellcheck disable=SC2016 # expanded in the container
    in_container bash -c '
        colcon test "$@" --return-code-on-test-failure --event-handlers console_direct+
        status=$?
        colcon test-result --verbose
        exit "${status}"
    ' bash "${select[@]}"
}

case "${1:-help}" in
    start)
        # No X server (a remote session) is fine: only RViz needs it.
        xhost +local:docker >/dev/null 2>&1 || true
        ensure_image
        docker compose up -d --no-build "${SERVICE}"
        ;;
    stop) docker compose stop "${SERVICE}" ;;
    recreate)
        xhost +local:docker >/dev/null 2>&1 || true
        ensure_image
        docker compose up -d --no-build --force-recreate "${SERVICE}"
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
    format) format_sources ;;
    lint) lint ;;
    help | -h | --help) usage ;;
    *)
        usage >&2
        exit 1
        ;;
esac
