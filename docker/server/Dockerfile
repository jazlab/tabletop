ARG ROS_DISTRO=jazzy

# Base image for running all server nodes
FROM ros:$ROS_DISTRO AS base

SHELL ["/bin/bash", "-c"]

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Add SR Research repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
<<EOT
set -ex
if [[ $(uname -m) = x86_64 ]] ; then
    apt-get update && apt-get upgrade -y
    apt-get install -y ca-certificates software-properties-common
    apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
    add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
    apt-get update
    apt-get install -y eyelink-display-software
    apt-get clean && rm -rf /var/lib/apt/lists/*
fi
EOT

# Install apt packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=deps/apt.list,target=/tmp/apt.list \
<<EOT
set -ex
apt-get update && apt-get upgrade -y
apt-get install -y $(cat /tmp/apt.list | xargs)
EOT

# Create user if not root
ARG USER
ARG UID
ARG GID=$UID

RUN <<EOT
set -ex
if [[ -z $USER || $USER = root ]] ; then
    echo "USER must be set to a non-root user"
    exit 1
fi
if [[ -z $UID || -z $GID || $UID = 0 || $GID = 0 ]] ; then
    echo "UID and GID must be set and cannot be 0"
    exit 1
fi
if id -u $UID ; then
    echo "User ID $UID already exists, deleting..."
    userdel -r $(id -un $UID)
fi
groupadd --gid $GID $USER
useradd --uid $UID --gid $GID -m $USER
echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER
chmod 0440 /etc/sudoers.d/$USER
EOT

USER $USER

# Install ROS dependencies from src directory
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src,target=/tmp/src \
<<EOT
set -ex
cd /tmp/src
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
echo $DEPENDENCY_TYPES
DEPENDENCY_TYPES="-t buildtool_export -t build -t build_export -t buildtool -t exec"
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO $DEPENDENCY_TYPES -y
EOT

WORKDIR /tabletop

# Install python dependencies using uv
ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

ARG USE_NVIDIA
RUN --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
<<EOT
set -ex
if [[ $USE_NVIDIA = true ]] ; then
    UV_EXTRA="--extra cu128"
else
    UV_EXTRA="--extra cpu"
fi
if [[ $(uname -m) = x86_64 ]] ; then
    UV_EXTRA="$UV_EXTRA --extra eyelink"
fi
uv sync --locked --no-install-project $UV_EXTRA
EOT

# Install colcon mixins
RUN <<EOT
set -ex
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
EOT

ENV TT_IN_CONTAINER=true

# Development image with additional dependencies and tools
FROM base AS dev

RUN <<EOT
set -ex
curl -fsSL -o ~/complete-alias \
https://raw.githubusercontent.com/cykerway/complete-alias/refs/heads/master/complete_alias
chmod +x ~/complete-alias
echo ". ~/complete-alias" >> ~/.bash_completion
EOT

# Install starship
RUN curl -sS https://starship.rs/install.sh | sh -s -- -y

# Install platformio
RUN <<EOT
set -ex
curl -fsSL -o /tmp/get-platformio.py \
https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
mkdir -p ~/.local/bin
ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio
ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio
ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb
rm /tmp/get-platformio.py
EOT

# Update .bashrc (must be unindented)
RUN <<EOT
set -ex
cat <<EOF >> ~/.bashrc
export EDITOR=vim
export PATH=~/.local/bin:\$PATH
source /tabletop/setup.bash
eval "\$(uv generate-shell-completion bash)"
eval "\$(uvx --generate-shell-completion bash)"
eval "\$(starship init bash)"
complete -F _complete_alias "\${!BASH_ALIASES[@]}"
EOF
mkdir -p ~/.config
EOT
