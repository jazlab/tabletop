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
export COMPOSE_BAKE=true

# Update PATH
export PATH=$TABLETOP_DIR/bin:$PATH

# Aliases
alias tree="tree -I 'build|install|logs|results"

# ROS-specific
if [ -d /opt/ros ]; then
    alias tt-server="ros2 launch tabletop_server server.launch.py"
    alias tt-commander="ros2 launch tabletop_server commander.launch.py"
    alias tt-tasks="ros2 launch tabletop_tasks tasks.launch.py"
fi

# Source .venv if it exists
if [ -d $TABLETOP_DIR/.venv ]; then
    export PYTHONPATH=$TABLETOP_DIR/.venv/lib/site-packages:$PYTHONPATH
fi
