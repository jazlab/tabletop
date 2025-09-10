# Utility functions
get_parent_dir() {
    # Check if the correct number of arguments are provided
    if [ $# -ne 2 ]; then
        print_error "Usage: get_parent_dir <path> <n>"
        exit 1
    fi
    local path=$1
    local n=$2

    # If path is a file, start from its directory
    if [ -f "$path" ]; then
        path=$(dirname $path)
    fi

    # Move up n directories
    for ((i=0; i<n; i++)); do
        path=$(dirname $path)
    done

    echo $path
}

print_status() {
    echo -e "\033[1;34m$@\033[0m"
}

print_error() {
    echo -e "\033[1;31m$@\033[0m"
}

print_warning() {
    echo -e "\033[1;33m$@\033[0m"
}


# Aliases
if [ -d /opt/ros ]; then
    alias tt-server="ros2 launch tabletop_server server.launch.py"
    alias tt-commander="ros2 launch tabletop_server commander.launch.py"
    alias tt-tasks="ros2 launch tabletop_tasks tasks.launch.py"
    alias tree="tree -I 'build|install|logs|results"
fi

# Set environment variables
export TABLETOP_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
export COLCON_WS=$TABLETOP_DIR
export ROS_LOG_DIR=$TABLETOP_DIR/log
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

# Add tabletop bin directory to PATH
export PATH=$TABLETOP_DIR/bin:$PATH

# Set build variables for Docker
export TT_UID=$(id -u)
export COMPOSE_BAKE=true
if command -v nvidia-smi >/dev/null 2>&1; then
    export TT_USE_NVIDIA=true
    export TT_CONTAINER_RUNTIME=nvidia
    export TT_NVIDIA_VISIBLE_DEVICES=all
    export TT_NVIDIA_DRIVER_CAPABILITIES=all
    export TT_UV_EXTRA="--extra cu128"
else
    export TT_USE_NVIDIA=false
    export TT_CONTAINER_RUNTIME=runc
    export TT_NVIDIA_VISIBLE_DEVICES=
    export TT_NVIDIA_DRIVER_CAPABILITIES=
    export TT_UV_EXTRA="--extra cpu"
fi

if [ "$(uname -m)" = "x86_64" ] ; then
    export TT_EYELINK_SUPPORTED=true
    export TT_UV_EXTRA="$TT_UV_EXTRA --extra eyelink"
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
