# Directory configuration
export TABLETOP_DIR="${TABLETOP_DIR:-$(dirname $(realpath ${BASH_SOURCE[0]}))}"
export TABLETOP_CACHE_DIR="${TABLETOP_CACHE_DIR:-$TABLETOP_DIR/.cache/tabletop}"
export COLCON_WS="${COLCON_WS:-$TABLETOP_DIR}"
export COLCON_LOG_DIR="${COLCON_LOG_DIR:-$COLCON_WS/log/colcon}"
export CCACHE_DIR="${CCACHE_DIR:-$TABLETOP_DIR/.cache/ccache}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$TABLETOP_DIR/log/ros}"
export ROS_BAG_DIR="${ROS_BAG_DIR:-$TABLETOP_DIR/bags}"

# ROS configuration
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
# export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"

# Foxglove configuration
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# UR configuration
export ROBOT_IP="${ROBOT_IP:-192.168.13.20}"
export SIM_ROBOT_IP="${SIM_ROBOT_IP:-192.168.12.20}"
export LEFT_ROBOT_IP="${LEFT_ROBOT_IP:-192.168.13.21}"
export RIGHT_ROBOT_IP="${RIGHT_ROBOT_IP:-192.168.13.20}"
export LEFT_SIM_ROBOT_IP="${LEFT_SIM_ROBOT_IP:-192.168.12.20}"
export RIGHT_SIM_ROBOT_IP="${RIGHT_SIM_ROBOT_IP:-192.168.12.20}"
export REVERSE_IP="${REVERSE_IP:-192.168.13.10}"
export SIM_REVERSE_IP="${SIM_REVERSE_IP:-192.168.12.10}"

# Source Python virtual environment
if [ -f "$TABLETOP_DIR/.venv/bin/activate" ]; then
    source "$TABLETOP_DIR/.venv/bin/activate"
fi

# Source colcon cd and argcomplete if it exists
if [ -f "$TABLETOP_DIR/.venv/share/colcon_cd/function/colcon_cd.sh" ]; then
    source "$TABLETOP_DIR/.venv/share/colcon_cd/function/colcon_cd.sh"
fi
if [ -f "$TABLETOP_DIR/.venv/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
    source "$TABLETOP_DIR/.venv/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi

# Add Spinnaker bin directory to PATH
# export PATH="/opt/spinnaker/bin:$PATH
# export SPINNAKER_GENTL64_CTI=/opt/ros/$ROS_DISTRO/lib/spinnaker-gentl/Spinnaker_GenTL.cti
# export SPINNAKER_GENTL64_CTI=$COLCON_WS/install/spinnaker_camera_driver/lib/spinnaker-gentl/Spinnaker_GenTL.cti

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    if [ -f "$COLCON_WS/install/setup.bash" ]; then
        source "$COLCON_WS/install/setup.bash"
    else
        source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    fi
fi

# Add correct bin directories to PATH based on whether or not setup.bash was called inside the container or not
export PATH="$TABLETOP_DIR/bin/common:$PATH"
if [ "$TABLETOP_CONTAINER" = "true" ]; then
    export PATH="$TABLETOP_DIR/bin/container:$PATH"
else
    export PATH="$TABLETOP_DIR/bin/host:$PATH"
fi
