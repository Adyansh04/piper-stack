# The piper-stack image: ROS 2 Lyrical (Ubuntu 26.04, GCC 15) with everything the stack runs:
# ros2_control, MoveIt 2, the wrist camera's driver, CUDA and TensorRT for the perception engines,
# Node for the behaviour-tree web UI, and uv for the Python 3.12 environments of the model servers.
# The source is mounted at /piper-stack, never copied in.
ARG BASE_IMAGE=ros:lyrical-ros-base@sha256:3224ee150cb02eb89a5e0811bc267b57ed3a91b9c10191df57810726a7e916d7
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Toolchain, and the ROS packages the arm, MoveIt, the behaviour trees and recording run on.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build mold ccache gdb \
        clang-format clang-tidy clangd \
        can-utils iproute2 curl wget ca-certificates git \
        libzmq3-dev cppzmq-dev zlib1g-dev nodejs npm \
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
        ros-lyrical-moveit \
        ros-lyrical-moveit-servo \
        ros-lyrical-moveit-planners-ompl \
        ros-lyrical-pilz-industrial-motion-planner \
        ros-lyrical-moveit-ros-perception \
        ros-lyrical-moveit-configs-utils \
        ros-lyrical-moveit-setup-assistant \
        ros-lyrical-pick-ik \
        ros-lyrical-behaviortree-cpp \
        ros-lyrical-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# The wrist camera: Percipio's camport_ros2 driver (ros_ws/src/vendor) and an image viewer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-camera-info-manager \
        ros-lyrical-cv-bridge \
        ros-lyrical-diagnostic-updater \
        ros-lyrical-example-interfaces \
        ros-lyrical-image-publisher \
        ros-lyrical-image-transport \
        ros-lyrical-rqt-image-view \
        ros-lyrical-tf2-eigen \
        ros-lyrical-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# CUDA 13 and TensorRT 11 from NVIDIA's Ubuntu 26.04 repository: SAM 3.1 and the policies run as
# TensorRT engines from C++.
ARG CUDA_VERSION=13-4
ARG TENSORRT_VERSION=11.4.0.106-1+cuda13.4
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2604/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
        cuda-nvcc-${CUDA_VERSION} \
        cuda-cudart-dev-${CUDA_VERSION} \
        libnvinfer11=${TENSORRT_VERSION} \
        libnvinfer-headers-dev=${TENSORRT_VERSION} \
        libnvinfer-dev=${TENSORRT_VERSION} \
        libnvonnxparsers11=${TENSORRT_VERSION} \
        libnvonnxparsers-dev=${TENSORRT_VERSION} \
        libnvinfer-bin=${TENSORRT_VERSION} \
    && rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/cuda/bin:${PATH}

# uv, which installs Python 3.12 and the locked environments of the model servers (GraspGenX).
ARG UV_VERSION=0.13.5
RUN curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" | env UV_INSTALL_DIR=/usr/local/bin sh

# The GraspGenX server's Python 3.12 environment, from its locked project (graspgenx/): GraspGenX's
# torch has no wheels for Lyrical's Python 3.14. The import check points GraspGenX's setup hook at
# existing directories, or it would clone its checkpoints from Hugging Face during the build.
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
COPY graspgenx/pyproject.toml graspgenx/uv.lock /opt/graspgenx/
RUN UV_PROJECT_ENVIRONMENT=/opt/graspgenx-venv uv sync --project /opt/graspgenx --locked --no-cache \
        --python 3.12 \
    && GRASPGENX_CHECKPOINT_DIR=/opt/graspgenx GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx \
        /opt/graspgenx-venv/bin/python -c "import graspgenx.x_grippers, torch, zmq, yaml"

# Linters for the shell scripts and the Python (scripts/manage.sh lint).
ARG RUFF_VERSION=0.17.4
RUN apt-get update && apt-get install -y --no-install-recommends shellcheck \
    && rm -rf /var/lib/apt/lists/* \
    && UV_TOOL_DIR=/opt/uv-tools UV_TOOL_BIN_DIR=/usr/local/bin uv tool install "ruff==${RUFF_VERSION}"

# The simulator's ros2_control hardware (hardware:=isaac): joint commands and states over topics.
# Late in the file, so adding it did not rebuild the layers above.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-joint-state-topic-hardware-interface \
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
