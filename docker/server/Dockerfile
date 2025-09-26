ARG ROS_DISTRO=jazzy

# Base image for running all server nodes
FROM ros:$ROS_DISTRO AS base

SHELL ["/bin/bash", "-c"]

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

ARG TARGETARCH

# Install Eyelink Display Software
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    <<EOT
set -ex
if [[ $TARGETARCH = amd64 ]] ; then
    apt-get update && apt-get upgrade -y
    apt-get install -y ca-certificates software-properties-common
    apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
    add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
    apt-get update
    apt-get install -y eyelink-display-software
fi
EOT

ARG USE_NVIDIA

# Install CUDA Toolkit
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    <<EOT
set -ex
cd /tmp
if [[ $USE_NVIDIA = true ]] ; then
    if [[ $TARGETARCH != amd64 ]] ; then
        echo "Unsupported architecture when using nvidia: $TARGETARCH!"
        exit 1
    fi
    apt-get update && apt-get upgrade -y
    apt-get install -y wget
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
    dpkg -i cuda-keyring_1.1-1_all.deb
    apt-get update
    apt-get -y install cuda-toolkit-12-9
    rm cuda-keyring_1.1-1_all.deb
fi
EOT

# Install Spinnaker SDK
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#     --mount=type=cache,target=/var/lib/apt,sharing=locked \
# <<EOT
# set -ex
# apt-get update && apt-get upgrade -y
# apt-get install -y curl
# cd /tmp
# case $TARGETARCH in
#     amd64)
#         curl -fsSL -o spinnaker.tar.gz https://flir.netx.net/file/asset/71711/original/attachment
#         ;;
#     arm64)
#         curl -fsSL -o spinnaker.tar.gz https://flir.netx.net/file/asset/71712/original/attachment
#         ;;
#     *)
#         echo "Unsupported architecture $TARGETARCH!"
#         exit 1
#         ;;
# esac
# tar -xzf spinnaker.tar.gz
# SPINNAKER_DIR=$(tar -tf spinnaker.tar.gz | head -1)
# cd $SPINNAKER_DIR
# yes | sudo apt install ./*.deb
# rm -rf /tmp/spinnaker.tar.gz
# EOT

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
ARG GID
ARG GROUPS

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

if id -u $UID >/dev/null 2>&1; then
    echo "User ID $UID already exists, deleting..."
    userdel -r $(id -un $UID)
fi
if ! getent group $GID >/dev/null 2>&1 ; then
    echo "Group ID $GID does not exist, creating..."
    groupadd -g $GID $USER
fi
useradd -u $UID -g $GID -s /bin/bash -G $GROUPS -m $USER
echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER
chmod 0440 /etc/sudoers.d/$USER
EOT

USER $USER

# Install ROS dependencies from src/ros directory
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,source=src/ros,target=/tmp/src \
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

WORKDIR /home/$USER/tabletop

# Install python dependencies using uv
ENV UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_DOWNLOADS=0

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

RUN --mount=type=cache,target=/home/$USER/.cache/uv,uid=$UID,gid=$GID \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    <<EOT
set -ex
if [[ $USE_NVIDIA = true ]] ; then
    UV_EXTRA="--extra cu129"
else
    UV_EXTRA="--extra cpu"
fi
if [[ $TARGETARCH = amd64 ]] ; then
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

# Install Neovim
RUN <<EOT
set -ex
case $TARGETARCH in
    amd64)
        curl -fsSL -o /tmp/nvim.tar.gz https://github.com/neovim/neovim/releases/download/stable/nvim-linux-x86_64.tar.gz
        ;;
    arm64)
        curl -fsSL -o /tmp/nvim.tar.gz https://github.com/neovim/neovim/releases/download/stable/nvim-linux-arm64.tar.gz
        ;;
    *)
        echo "Unsupported architecture $TARGETARCH!"
        exit 1
        ;;
esac
sudo tar -xzf /tmp/nvim.tar.gz -C /opt
NVIM_DIRNAME=$(tar -tf /tmp/nvim.tar.gz | head -1 | cut -f1 -d"/")
sudo ln -s /opt/$NVIM_DIRNAME/bin/nvim /usr/bin/nvim
rm -f /tmp/nvim.tar.gz
EOT

# Install npm
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    <<EOT
set -ex
apt-get update && apt-get upgrade -y
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
\. "$HOME/.nvm/nvm.sh"
nvm install 22
EOT

# Install complete-alias
RUN <<EOT
set -ex
curl -fsSL -o ~/complete-alias \
https://raw.githubusercontent.com/cykerway/complete-alias/refs/heads/master/complete_alias
chmod +x ~/complete-alias
echo ". ~/complete-alias" >> ~/.bash_completion
EOT

# Install kitten
RUN <<EOT
set -ex
case $TARGETARCH in
    amd64)
        sudo curl -fsSL -o /usr/local/bin/kitten https://github.com/kovidgoyal/kitty/releases/latest/download/kitten-linux-amd64
        ;;
    arm64)
        sudo curl -fsSL -o /usr/local/bin/kitten https://github.com/kovidgoyal/kitty/releases/latest/download/kitten-linux-arm64
        ;;
    *)
        echo "Unsupported architecture $TARGETARCH!"
        exit 1
        ;;
esac
sudo chmod +x /usr/local/bin/kitten
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
export PATH=~/.local/bin:\$PATH
source /home/$USER/tabletop/setup.bash
eval "\$(uv generate-shell-completion bash)"
eval "\$(uvx --generate-shell-completion bash)"
alias vim="nvim"
export EDITOR=nvim
complete -F _complete_alias "\${!BASH_ALIASES[@]}"
eval "\$(starship init bash)"
EOF
mkdir -p ~/.config/pulse
EOT
