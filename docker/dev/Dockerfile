# The piper-stack image: ROS 2 Lyrical (Ubuntu 26.04, GCC 15) with everything the stack runs:
# ros2_control, MoveIt 2, the wrist camera's driver, CUDA and TensorRT for the perception engines,
# and uv for the Python 3.12 environments of the model servers.
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
        libzmq3-dev cppzmq-dev zlib1g-dev \
        python3-colcon-common-extensions python3-rosdep \
        ros-lyrical-ros2-control \
        ros-lyrical-ros2-controllers \
        ros-lyrical-joint-state-topic-hardware-interface \
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

# CUDA 13 and TensorRT 11 from NVIDIA's Ubuntu 26.04 repository: SAM 3.1 runs as TensorRT
# engines from C++.
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

# PlotJuggler 4 for live plots and bags (layouts in piper_bringup/config/plotjuggler), with its
# official ROS 2 topic subscriber, ROS message parser and MCAP loader, each checked by sha256. Its
# apt repository serves amd64 only: an arm64 image goes without.
ARG PLOTJUGGLER_VERSION=4.1.0-1
ARG PJ_PLUGINS=https://github.com/PlotJuggler/pj-official-plugins/releases/download
RUN if [ "$(dpkg --print-architecture)" = amd64 ]; then \
        install -d -m 0755 /etc/apt/keyrings \
        && curl -fsSL https://apt.plotjuggler.io/plotjuggler-archive-keyring.asc \
            -o /etc/apt/keyrings/plotjuggler.asc \
        && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/plotjuggler.asc] https://apt.plotjuggler.io stable main" \
            > /etc/apt/sources.list.d/plotjuggler.list \
        && apt-get update \
        && apt-get install -y --no-install-recommends "plotjuggler4=${PLOTJUGGLER_VERSION}" unzip \
        && rm -rf /var/lib/apt/lists/* \
        && ext=/home/ubuntu/.local/share/PlotJuggler/PlotJuggler4/extensions && mkdir -p "${ext}" \
        && fetch() { curl -fsSL -o /tmp/extension.zip "${PJ_PLUGINS}/$1" \
            && echo "$2  /tmp/extension.zip" | sha256sum -c - \
            && unzip -q /tmp/extension.zip -d "${ext}" && rm /tmp/extension.zip; } \
        && fetch data_stream_ros2%2Fv1.1.3/ros2-topic-subscriber-1.1.3-linux-x86_64.zip \
            5eae8410957081cfda8ed431d0bc4f62e3c2e57100e48d83f9e6cd70c7c73465 \
        && fetch parser_ros%2Fv1.3.1/ros-parser-1.3.1-linux-x86_64.zip \
            62483526525c036ef59f6564ba30501e34c660bdd4de17cc843fa07f4bf40df0 \
        && fetch data_load_mcap%2Fv1.3.0/mcap-loader-1.3.0-linux-x86_64.zip \
            d58dc3ab1af06783d8bf0ce3a488b18ff480ac3fbea4f6e2425d30230eee4038; \
    fi

# ONNX Runtime (CPU) for the policy controller in piper_controllers, which runs trained policies
# inside the control loop.
RUN apt-get update && apt-get install -y --no-install-recommends ros-lyrical-onnxruntime-vendor \
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
