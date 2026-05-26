ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

FROM ros:jazzy

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
    git \
    bash-completion \
    usbutils \
    nodejs \
    npm \
    python3-pip \
    python3-venv \
    python-is-python3 \
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
    apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
    add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
    apt-get update
    apt-get install -y --no-install-recommends eyelink-display-software
fi
EOT

# Install ROS dependencies from src/ros directory
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src/ros,target=/tmp/src <<EOT
set -ex
source /opt/ros/$ROS_DISTRO/setup.bash
apt-get update
rosdep update --rosdistro $ROS_DISTRO
DEPENDENCY_TYPES="-t buildtool_export -t build -t build_export -t buildtool -t exec"
rosdep install -r --from-paths /tmp/src --ignore-src --rosdistro $ROS_DISTRO $DEPENDENCY_TYPES -y
EOT

# Set colcon home directory to /colcon and copy colcon mixins from /root to /colcon
ENV COLCON_HOME=/colcon

RUN cp -r /root/.colcon /colcon && \
    chmod -R 777 /colcon

# Install platformio to /platformio directory
ENV PLATFORMIO_CORE_DIR=/platformio

RUN <<EOT
set -ex
curl -fsSL -o /tmp/get-platformio.py \
https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
rm /tmp/get-platformio.py
chmod -R 777 /platformio
ln -s /platformio/penv/bin/platformio /bin/platformio
ln -s /platformio/penv/bin/pio /bin/pio
ln -s /platformio/penv/bin/piodebuggdb /bin/piodebuggdb
EOT

# Add uv binaries
COPY --from=ghcr.io/astral-sh/uv@sha256:88234bc9e09c2b2f6d176a3daf411419eb0370d450a08129257410de9cfafd2a \
    /uv /uvx /bin/

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0

# Copy entrypoint script
COPY docker/ros/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

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

# Create ~/.config directory for pulse bind mount
RUN mkdir -p /home/$USER_NAME/.config

# Variable to indicate we are in the tabletop container
ENV TABLETOP_CONTAINER=true
