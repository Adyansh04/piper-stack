# The piper-stack dev image: ROS 2 Lyrical (Ubuntu 26.04, GCC 15), the build and lint tools, and
# the ROS packages the stack runs on. The source is mounted at /piper-stack, never copied in.
ARG BASE_IMAGE=ros:lyrical-ros-base@sha256:3224ee150cb02eb89a5e0811bc267b57ed3a91b9c10191df57810726a7e916d7
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build mold ccache gdb \
        clang-format clang-tidy clangd \
        can-utils iproute2 \
        python3-colcon-common-extensions python3-rosdep \
        ros-lyrical-ros2-control \
        ros-lyrical-ros2-controllers \
        ros-lyrical-realtime-tools \
        ros-lyrical-xacro \
        ros-lyrical-robot-state-publisher \
        ros-lyrical-joint-state-publisher-gui \
        ros-lyrical-rviz2 \
        ros-lyrical-rmw-cyclonedds-cpp \
        ros-lyrical-ament-cmake-gtest \
        ros-lyrical-ament-lint-auto \
    && rm -rf /var/lib/apt/lists/*

# The container runs as the host user so that files it writes in the checkout stay the user's: the
# base image's own user `ubuntu` takes the host's IDs.
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN groupmod -o -g "${HOST_GID}" ubuntu \
    && usermod -o -u "${HOST_UID}" -g "${HOST_GID}" ubuntu \
    && mkdir -p /home/ubuntu/.cache/ccache \
    && chown -R ubuntu:ubuntu /home/ubuntu \
    && printf '%s\n' 'source /opt/ros/lyrical/setup.bash' \
        '[[ -f /piper-stack/ros_ws/install/setup.bash ]] && source /piper-stack/ros_ws/install/setup.bash' \
        >> /home/ubuntu/.bashrc

USER ubuntu
WORKDIR /piper-stack/ros_ws
CMD ["sleep", "infinity"]
