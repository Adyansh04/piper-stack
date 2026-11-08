# The simulator image: Isaac Sim 6.2 with Isaac Lab 3.0 for training. The scene (sim/) and the
# tasks (lab/) are mounted at /piper-stack, never copied in. Isaac Sim talks to the Lyrical stack
# over DDS through its bundled Jazzy bridge. Not published: built where it runs.
ARG BASE_IMAGE=nvcr.io/nvidia/isaac-sim:6.2.0.1
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates libglib2.0-0 ncurses-term \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab at its release tag, in a uv venv on Kit's own Python, as its own image does: its
# packages resolve from uv.lock, and isaaclab.sh keeps Isaac Sim importable.
ARG ISAACLAB_REF=v3.0.0
ARG UV_VERSION=0.13.5
ENV ISAACSIM_ROOT_PATH=/isaac-sim \
    ISAACLAB_PATH=/isaaclab \
    VIRTUAL_ENV=/opt/isaaclab-venv \
    UV_PROJECT_ENVIRONMENT=/opt/isaaclab-venv \
    UV_PYTHON=/isaac-sim/kit/python/bin/python3 \
    UV_PYTHON_PREFERENCE=only-system \
    UV_HTTP_RETRIES=12
RUN curl -LsSf "https://astral.sh/uv/${UV_VERSION}/install.sh" | env UV_INSTALL_DIR=/usr/local/bin sh \
    && git clone --depth 1 --branch "${ISAACLAB_REF}" https://github.com/isaac-sim/IsaacLab.git "${ISAACLAB_PATH}" \
    && ln -sf "${ISAACSIM_ROOT_PATH}" "${ISAACLAB_PATH}/_isaac_sim" \
    && uv venv --seed "${VIRTUAL_ENV}" \
    && cd "${ISAACLAB_PATH}" \
    && uv sync --frozen --inexact --extra rsl-rl --no-install-package imageio-ffmpeg \
    && ./isaaclab.sh -p tools/install_deps.py apt source \
    && ./isaaclab.sh -p -c "from isaaclab.cli.commands.install import _repoint_prebundle_packages; _repoint_prebundle_packages()" \
    && rm -rf /var/lib/apt/lists/* /root/.cache/uv
# From here uv targets the venv. MuJoCo is only Newton's physics, never a renderer; OmniHub may
# start (the base image forbids it, and every Kit start then retries it for a while).
ENV UV_PYTHON=/opt/isaaclab-venv/bin/python \
    MUJOCO_GL=disable \
    HUB__ARGS__DETECT_ONLY=false

# The container runs as the host user (compose `user:`), so files it writes
# in the checkout stay the user's: the directories Kit writes are created open to any user.
ENV HOME=/home/sim
RUN mkdir -p /isaac-sim/kit/cache /isaac-sim/kit/logs /isaac-sim/kit/data /home/sim \
    && chmod 1777 /isaac-sim/kit/cache /isaac-sim/kit/logs /isaac-sim/kit/data /home/sim /var/cache/hub \
    && chmod 755 /isaac-sim

WORKDIR /piper-stack
ENTRYPOINT []
CMD ["sleep", "infinity"]
