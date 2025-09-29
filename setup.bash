# Set environment variables
export TABLETOP_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
export COLCON_WS=$TABLETOP_DIR
export COLCON_LOG_DIR=$COLCON_WS/log/colcon
export ROS_LOG_DIR=$TABLETOP_DIR/log/ros
export ROS_BAG_DIR=$TABLETOP_DIR/bags
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SIM_ROBOT_IP=192.168.12.20
export SIM_REVERSE_IP=192.168.12.10
export ROBOT_IP=192.168.13.20
export REVERSE_IP=192.168.13.10
export PYTHONUNBUFFERED=1

# Source Python virtual environment
if [ -f $TABLETOP_DIR/.venv/bin/activate ]; then
    source $TABLETOP_DIR/.venv/bin/activate
fi

# Source colcon cd and argcomplete if it exists
if [ -f $TABLETOP_DIR/.venv/share/colcon_cd/function/colcon_cd.sh ]; then
    source $TABLETOP_DIR/.venv/share/colcon_cd/function/colcon_cd.sh
fi
if [ -f $TABLETOP_DIR/.venv/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source $TABLETOP_DIR/.venv/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# Source ROS environment
if [ -f /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash ]; then
    # Source ROS environment
    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
    # Source colcon workspace
    if [ -f $COLCON_WS/install/setup.bash ]; then
        source $COLCON_WS/install/setup.bash
    fi
fi

# Add Spinnaker bin directory to PATH
# export PATH=/opt/spinnaker/bin:$PATH
export SPINNAKER_GENTL64_CTI=/opt/ros/$ROS_DISTRO/lib/spinnaker-gentl/Spinnaker_GenTL.cti

# Add correct bin directories to PATH based on whether or not setup.bash was called inside the container or not
export PATH=$TABLETOP_DIR/bin/common:$PATH
if [[ $TT_IN_CONTAINER = true ]]; then
    export PATH=$TABLETOP_DIR/bin/container:$PATH
else
    export PATH=$TABLETOP_DIR/bin/host:$PATH
fi

# Set build variables for Docker
export TT_USER=$(id -un)
export TT_UID=$(id -u)
export TT_GID=$(id -g)
export TT_DISPLAY_WIDTH=1920
export TT_DISPLAY_HEIGHT=1080
export COMPOSE_BAKE=true
if command -v nvidia-smi >/dev/null 2>&1; then
    export TT_USE_NVIDIA=true
    export TT_INSTALL_CUDA= # Used for building isaac cumotion library, if applicable
    export TT_CUDA_VERSION=129
    export TT_UV_EXTRA="--extra cu$TT_CUDA_VERSION"
    export TT_CONTAINER_RUNTIME=nvidia
    export TT_NVIDIA_VISIBLE_DEVICES=all
    export TT_NVIDIA_DRIVER_CAPABILITIES=all
else
    export TT_USE_NVIDIA=
    export TT_CONTAINER_RUNTIME=runc
    export TT_NVIDIA_VISIBLE_DEVICES=
    export TT_NVIDIA_DRIVER_CAPABILITIES=
    export TT_UV_EXTRA="--extra cpu"
fi

if [ "$(uname -m)" = "x86_64" ] ; then
    export TT_EYELINK_SUPPORTED=true
    export TT_UV_EXTRA="$TT_UV_EXTRA --extra eyelink"
fi

if [[ $(uname) = Darwin ]]; then
    export TT_BIND_CONSISTENCY=cached
else
    export TT_BIND_CONSISTENCY=consistent
fi

if command -v pactl >/dev/null 2>&1; then
    export TT_PULSE_SERVER_HOST=$(pactl --format=json info | jq -r '.server_string')
    if [[ -S $TT_PULSE_SERVER_HOST ]]; then
        export TT_PULSE_COOKIE_HOST=~/.config/pulse/cookie
    else
        export TT_PULSE_SERVER_HOST=/tmp/pulseaudio-empty.socket
        export TT_PULSE_COOKIE_HOST=/tmp/pulseaudio-empty.cookie
    fi
elif [ -S "$XDG_RUNTIME_DIR/pulse/native" ]; then
    export TT_PULSE_SERVER_HOST=$XDG_RUNTIME_DIR/pulse/native
    export TT_PULSE_COOKIE_HOST=~/.config/pulse/cookie
else
    export TT_PULSE_SERVER_HOST=/tmp/pulseaudio-empty.socket
    export TT_PULSE_COOKIE_HOST=/tmp/pulseaudio-empty.cookie
fi
