ARG BASE_IMAGE=ros:jazzy
ARG USER_NAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG USER_GROUPS
ARG UV_EXTRA

FROM $BASE_IMAGE

SHELL ["/bin/bash", "-c"]

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Upgrade existing packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y

# Install apt packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked <<EOT
set -ex
apt-get update
apt-get install -y \
    sudo \
    curl \
    wget \
    git \
    python3-pip \
    python3-venv \
    python-is-python3 \
    cmake \
    mold \
    ccache \
    ffmpeg \
    fluidsynth
EOT

ARG TARGETARCH

# Install Eyelink Display Software
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked <<EOT
set -ex
if [[ $TARGETARCH = amd64 ]] ; then
    apt-get update
    apt-get install -y ca-certificates software-properties-common
    apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
    add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
    apt-get update
    apt-get install -y eyelink-display-software
fi
EOT

# Create user
ARG USER_NAME
ARG USER_UID
ARG USER_GID
ARG USER_GROUPS

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
if [[ $USER_GROUPS ]]; then
    usermod -aG $USER_GROUPS $USER_NAME
fi
echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME
chmod 0440 /etc/sudoers.d/$USER_NAME
EOT

USER $USER_NAME

# Install ROS dependencies from src/ros directory

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src/ros,target=/tmp/src <<EOT
set -ex
cd /tmp/src
sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
echo $DEPENDENCY_TYPES
DEPENDENCY_TYPES="-t buildtool_export -t build -t build_export -t buildtool -t exec"
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO $DEPENDENCY_TYPES -y
EOT

# Set working directory to /tabletop
WORKDIR /tabletop

# Install python dependencies using uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

ARG UV_EXTRA

ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0 \
    UV_EXTRA=$UV_EXTRA

RUN --mount=type=cache,target=/tmp/uv.cache,uid=$USER_UID,gid=$USER_GID \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml <<EOT
set -ex
export UV_CACHE_DIR=/tmp/uv.cache
uv sync --locked --no-install-project $UV_EXTRA
mkdir -p ~/.cache
cp -r $UV_CACHE_DIR ~/.cache/uv
EOT

# Install colcon mixins
RUN <<EOT
set -ex
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
EOT

ENV PATH="/home/$USER_NAME/.local/bin:$PATH"

# Install platformio
RUN --mount=type=bind,source=src/ros/tabletop/tabletop_teensy/platformio.ini,target=/tmp/platformio.ini <<EOT
set -ex
curl -fsSL -o /tmp/get-platformio.py \
https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
mkdir -p ~/.local/bin
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
rm /tmp/get-platformio.py
pio pkg install --global --project-dir /tmp
EOT

# Copy entrypoint script
COPY docker/ros/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Update .bashrc and create ~/.config directory for pulse bind mount
RUN <<EOT
set -ex
cat <<EOF >> ~/.bashrc
if [[ -f /tabletop/setup.bash ]]; then
    source /tabletop/setup.bash
else
    source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi
EOF
mkdir -p ~/.config
EOT

# Convenience variable to indicate we're in a container
ENV TABLETOP_CONTAINER=true
