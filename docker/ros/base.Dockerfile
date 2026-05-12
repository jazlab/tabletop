ARG BASE_IMAGE=ros:jazzy
ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG UV_EXTRA

# Install python dependencies using uv
FROM $BASE_IMAGE AS builder-uv

COPY --from=ghcr.io/astral-sh/uv@sha256:88234bc9e09c2b2f6d176a3daf411419eb0370d450a08129257410de9cfafd2a \
    /uv /uvx /bin/

ARG UV_EXTRA

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0 \
    UV_EXTRA=$UV_EXTRA \
    UV_CACHE_DIR=/uv-cache

WORKDIR /tabletop

RUN --mount=type=cache,target=/uv-cache \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --locked --no-install-project $UV_EXTRA

RUN --mount=type=cache,target=/uv-cache \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    --mount=type=bind,source=src/tabletop_py,target=src/tabletop_py \
    uv sync --locked $UV_EXTRA


FROM $BASE_IMAGE

SHELL ["/bin/bash", "-c"]

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install apt packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    software-properties-common \
    sudo \
    curl \
    wget \
    git \
    git-lfs \
    nodejs \
    npm \
    python3-pip \
    python3-venv \
    python-is-python3 \
    cmake \
    mold \
    ccache \
    ffmpeg \
    fluidsynth

ARG TARGETARCH

# Install Eyelink Display Software
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked <<EOT
set -ex
if [[ $TARGETARCH = amd64 ]] ; then
    apt-get update
    apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
    add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
    apt-get update
    apt-get install -y --no-install-recommends eyelink-display-software
fi
EOT

# Create user
ARG USER_NAME
ARG USER_UID
ARG USER_GID

RUN <<EOT
set -ex
if [[ -z $USER_NAME || $USER_NAME = root ]] ; then
    echo "USER_NAME must be set to a non-root user"
    exit 1
fi
if [[ -z $USER_UID || -z $USER_GID || $USER_UID = 0 || $USER_GID = 0 ]] ; then
    echo "USER_UID and USER_GID must be set and cannot be 0"
    exit 1
fi
if id -un $USER_NAME >/dev/null 2>&1; then
    echo "Username $USER_NAME already exists, deleting..."
    userdel -r $USER_NAME
fi
if id -u $USER_UID >/dev/null 2>&1; then
    echo "User ID $USER_UID already exists, deleting..."
    userdel -r $(id -un $USER_UID)
fi
if ! getent group $USER_GID >/dev/null 2>&1 ; then
    echo "Group ID $USER_GID does not exist, creating..."
    groupadd -g $USER_GID $USER_NAME
fi
useradd -u $USER_UID -g $USER_GID -s /bin/bash -m $USER_NAME
echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME
chmod 0440 /etc/sudoers.d/$USER_NAME
EOT

USER $USER_NAME

ENV PATH="/home/$USER_NAME/bin:/home/$USER_NAME/.local/bin:$PATH"

# Set working directory to /tabletop
WORKDIR /tabletop

# Install ROS dependencies from src/ros directory
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src/ros,target=src/ros <<EOT
set -ex
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt-get update
rosdep update --rosdistro $ROS_DISTRO
DEPENDENCY_TYPES="-t buildtool_export -t build -t build_export -t buildtool -t exec"
rosdep install -r --from-paths src/ros --ignore-src --rosdistro $ROS_DISTRO $DEPENDENCY_TYPES -y
EOT

# Install platformio
RUN --mount=type=bind,source=src/ros/tabletop/tabletop_teensy/platformio.ini,target=platformio.ini <<EOT
set -ex
curl -fsSL -o get-platformio.py \
https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
mkdir -p ~/.local/bin
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
rm get-platformio.py
pio pkg install --global
EOT

# Copy entrypoint script
COPY docker/ros/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Update .bashrc
RUN cat <<EOF >> ~/.bashrc
if [ -s /tabletop/setup.bash ]; then
    source /tabletop/setup.bash
else
    source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi
EOF

# Create ~/.config directory for pulse bind mount
RUN mkdir -p ~/.config

COPY --from=builder-uv /bin/uv /bin/uvx /bin/
COPY --from=builder-uv --chown=${USER_UID}:${USER_GID} /tabletop/.venv /tabletop/.venv

ARG UV_EXTRA

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0 \
    UV_EXTRA=$UV_EXTRA \
    PATH="/tabletop/.venv/bin:$PATH"

# Install colcon mixins
RUN <<EOT
set -ex
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
EOT

# Convenience variable to indicate we're in a container
ENV TABLETOP_CONTAINER=true
