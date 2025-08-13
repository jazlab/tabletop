ARG ROS_DISTRO=jazzy

# Base image for running all server nodes
FROM ros:$ROS_DISTRO AS base

ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

# Keep downloaded packages to reduce build time
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Add mirror to apt sources
RUN <<EOT
set -ex
cat <<EOF > /etc/apt/sources.list.d/mirrors.sources
Types: deb
URIs: http://us.archive.ubuntu.com/ubuntu/
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
apt-get clean && rm -rf /var/lib/apt/lists/*
EOT

# Add SR Research repository
RUN <<EOT
set -ex
apt-get update && apt-get upgrade -y
apt-get install -y ca-certificates software-properties-common
apt-key adv --fetch-keys https://apt.sr-research.com/SRResearch_key
add-apt-repository 'deb [arch=amd64] https://apt.sr-research.com SRResearch main'
apt-get clean && rm -rf /var/lib/apt/lists/*
EOT

# Install apt packages
RUN --mount=type=bind,source=apt.list,target=/tmp/apt.list \
    --mount=type=cache,target=/var/cache/apt,id=server-apt-cache,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,id=server-apt-lib,sharing=locked \
<<EOT
set -ex
apt-get update && apt-get upgrade -y
apt-get install -y $(cat /tmp/apt.list | xargs)
EOT

# Create user if not root
ARG USER=root
ARG UID=0
ARG GID=$UID

RUN <<EOT
set -ex
if [[ $USER != root ]] ; then
    if [[ $UID = 0 || $GID = 0 ]] ; then
        echo "UID and/or GID are 0 but USER is not root!"
        exit 1
    fi
    if id -u $UID ; then
        userdel -r $(id -un $UID)
    fi
    groupadd --gid $GID $USER
    useradd --uid $UID --gid $GID -m $USER
    echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER
    chmod 0440 /etc/sudoers.d/$USER
else
    if [[ $UID != 0 || $GID != 0 ]] ; then
        echo "UID and/or GID are not 0 but USER is root!"
        exit 1
    fi
fi
EOT

USER $USER

# Install pip dependencies into a virtual environment
RUN --mount=type=bind,source=requirements.txt,target=/tmp/requirements.txt \
    --mount=type=cache,target=/tmp/pip-cache,id=server-pip-cache,uid=$UID,gid=$GID \
<<EOT
set -ex
rm -rf ~/.cache/pip
mkdir -p ~/.cache
ln -s /tmp/pip-cache ~/.cache/pip
python3 -m venv --system-site-packages --symlinks ~/venv
source ~/venv/bin/activate
pip install -r /tmp/requirements.txt
rm ~/.cache/pip
EOT

# Install ROS dependencies from src directory
RUN --mount=type=bind,source=src,target=/tmp/src \
    --mount=type=cache,target=/var/cache/apt,id=server-apt-cache,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,id=server-apt-lib,sharing=locked \
<<EOT
set -ex
cd /tmp/src
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y
source ~/venv/bin/activate
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
EOT

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

# Install pip development dependencies into the virtual environment
RUN --mount=type=bind,source=requirements-dev.txt,target=/tmp/requirements-dev.txt \
    --mount=type=cache,target=/tmp/pip-cache,id=server-pip-cache,uid=$UID,gid=$GID \
<<EOT
set -ex
rm -rf ~/.cache/pip
mkdir -p ~/.cache
ln -s /tmp/pip-cache ~/.cache/pip
source ~/venv/bin/activate
pip install -r /tmp/requirements-dev.txt
rm ~/.cache/pip
EOT

# Update .bashrc (must be unindented)
RUN <<EOT
set -ex
cat <<EOF >> ~/.bashrc
export PATH=~/.local/bin:\$PATH
source ~/venv/bin/activate
source /opt/ros/$ROS_DISTRO/setup.bash
source /tabletop/install/setup.bash
source /tabletop/setup.bash
eval "\$(pip completion --bash)"
eval "\$(starship init bash)"
complete -F _complete_alias "\${!BASH_ALIASES[@]}"
EOF
EOT

VOLUME /tabletop
