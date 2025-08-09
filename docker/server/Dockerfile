ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO AS base

ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

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
RUN --mount=type=bind,source=apt.list,target=/tmp/apt.list <<EOT
    set -ex
    apt-get update && apt-get upgrade -y
    apt-get install -y $(cat /tmp/apt.list | xargs)
    apt-get clean && rm -rf /var/lib/apt/lists/*
EOT

# Install pip packages
RUN --mount=type=bind,source=requirements.txt,target=/tmp/requirements.txt \
    pip install --break-system-packages -r /tmp/requirements.txt

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
    fi
EOT

USER $USER

# Install ros dependencies from workspace (needs to be after user creation
# because of rosdep)
ARG SRC_DIR

RUN --mount=type=bind,source=$SRC_DIR,target=/tmp/src <<EOT
    set -ex
    cd /tmp/src
    sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y
    source /opt/ros/$ROS_DISTRO/setup.bash
    rosdep update
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
EOT

FROM base AS dev

SHELL ["/bin/bash", "-c"]

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

RUN --mount=type=bind,source=,target=/tmp/src <<EOT

# Update .bashrc (must be unindented)
RUN <<EOT
set -ex
cat <<EOF >> ~/.bashrc
export PATH=~/.local/bin:\$PATH
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ws/install/setup.bash
source ~/ws/src/tabletop/setup.bash
eval "\$(starship init bash)"
EOF
EOT
