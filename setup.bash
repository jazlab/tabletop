# Setup environment for TableTop ROS 2 project
# Sourced by scripts and containers to configure paths, Python venv, ROS installation, and CLI tools

# === Directory Configuration ===
# Core paths for workspace and build artifacts
export TABLETOP_DIR="${TABLETOP_DIR:-$(dirname $(realpath ${BASH_SOURCE[0]}))}"
export TABLETOP_CACHE_DIR="${TABLETOP_CACHE_DIR:-$TABLETOP_DIR/.cache/tabletop}"
export COLCON_WS="${COLCON_WS:-$TABLETOP_DIR}"
export COLCON_LOG_DIR="${COLCON_LOG_DIR:-$COLCON_WS/log/colcon}"
export CCACHE_DIR="${CCACHE_DIR:-$TABLETOP_DIR/.cache/ccache}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$TABLETOP_DIR/log/ros}"
export ROS_BAG_DIR="${ROS_BAG_DIR:-$TABLETOP_DIR/bags}"

# === Python Runtime Configuration ===
# Unbuffered output for real-time logging
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

# === ROS Middleware Configuration ===
# FastRTPS as default DDS middleware
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

# === ROS Logging Configuration ===
# Log to stdout and disable buffering for real-time monitoring
export RCUTILS_LOGGING_USE_STDOUT="${RCUTILS_LOGGING_USE_STDOUT:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"
# export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"

# === Foxglove Bridge Configuration ===
# Port for visualization bridge
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# === Robot Network Configuration ===
# UR5e robot IP addresses and reverse SSH tunnel IPs
export ROBOT_IP="${ROBOT_IP:-192.168.13.20}"
export SIM_ROBOT_IP="${SIM_ROBOT_IP:-192.168.12.20}"
export LEFT_ROBOT_IP="${LEFT_ROBOT_IP:-192.168.13.21}"
export RIGHT_ROBOT_IP="${RIGHT_ROBOT_IP:-192.168.13.20}"
export LEFT_SIM_ROBOT_IP="${LEFT_SIM_ROBOT_IP:-192.168.12.20}"
export RIGHT_SIM_ROBOT_IP="${RIGHT_SIM_ROBOT_IP:-192.168.12.20}"
export REVERSE_IP="${REVERSE_IP:-192.168.13.10}"
export SIM_REVERSE_IP="${SIM_REVERSE_IP:-192.168.12.10}"

# === Python Virtual Environment Activation ===
# Detect container vs host and activate appropriate venv (container uses shared cache)
if [ "$TABLETOP_CONTAINER" = "true" ]; then
    export UV_CACHE_DIR="$TABLETOP_CACHE_DIR/uv-cache"
    export UV_PROJECT_ENVIRONMENT=".venv.container"
else
    export UV_PROJECT_ENVIRONMENT=".venv"
fi

if [ -f "$TABLETOP_DIR/$UV_PROJECT_ENVIRONMENT/bin/activate" ]; then
    source "$TABLETOP_DIR/$UV_PROJECT_ENVIRONMENT/bin/activate"
fi

# Add Spinnaker bin directory to PATH
# export PATH="/opt/spinnaker/bin:$PATH
# export SPINNAKER_GENTL64_CTI=/opt/ros/$ROS_DISTRO/lib/spinnaker-gentl/Spinnaker_GenTL.cti
# export SPINNAKER_GENTL64_CTI=$COLCON_WS/install/spinnaker_camera_driver/lib/spinnaker-gentl/Spinnaker_GenTL.cti

# === ROS 2 Environment Setup ===
# Source ROS installation (from colcon build artifacts if available, else system ROS)
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    if [ -f "$COLCON_WS/install/setup.bash" ]; then
        source "$COLCON_WS/install/setup.bash"
    else
        source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    fi
fi

# === Container-Specific Library Configuration ===
# (In containers only) Register ROS libraries in system loader cache for faster linking
if [ "$TABLETOP_CONTAINER" = "true" ]; then
    echo "$LD_LIBRARY_PATH" | tr ':' '\n' | sudo tee /etc/ld.so.conf.d/ros.conf > /dev/null
    sudo ldconfig
fi

# === Command-Line Tool PATH Setup ===
# Add TableTop CLI scripts to PATH (common scripts always available; host/container-specific scripts second)
export PATH="$TABLETOP_DIR/bin/common:$PATH"
if [ "$TABLETOP_CONTAINER" = "true" ]; then
    export PATH="$TABLETOP_DIR/bin/container:$PATH"
else
    export PATH="$TABLETOP_DIR/bin/host:$PATH"
fi

# # Source colcon cd and argcomplete if it exists
if [ -f "$VIRTUAL_ENV/share/colcon_cd/function/colcon_cd.sh" ]; then
    source "$VIRTUAL_ENV/share/colcon_cd/function/colcon_cd.sh"
fi
if [ -f "$VIRTUAL_ENV/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
    source "$VIRTUAL_ENV/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi
