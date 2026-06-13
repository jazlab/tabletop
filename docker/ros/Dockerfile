FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

USER root

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
    gosu \
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

# Copy entrypoint scripts
COPY docker/ros/entrypoint.sh /entrypoint.sh
COPY docker/ros/user-entrypoint.sh /user-entrypoint.sh
COPY docker/ros/fix-uid-gid.sh /fix-uid-gid.sh
ENTRYPOINT ["/entrypoint.sh"]

# Variable to indicate we are in the tabletop container
ENV TABLETOP_CONTAINER=true

RUN <<EOT
set -ex
userdel -r ubuntu
groupadd -g 1000 mules
useradd -u 1000 -g 1000 -s /bin/bash -m mules
echo "mules ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/mules
chmod 0440 /etc/sudoers.d/mules
EOT
