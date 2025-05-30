# Get bin directory and source utils
bin_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $bin_dir/utils.bash

# Set environment variables
export COLCON_WS=$(get_parent_dir $bin_dir 3)
export TABLETOP_DIR=$(get_parent_dir $bin_dir 1)
export ROS_LOG_DIR=$TABLETOP_DIR/ros/logs
export TABLETOP_BAG_DIR=$TABLETOP_DIR/ros/bags
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export PIP_BREAK_SYSTEM_PACKAGES=1

# Update PATH
export PATH=$HOME/.local/bin:$PATH
export PATH=$bin_dir:$PATH

# Source ROS environment
ros_distro=${ROS_DISTRO:-jazzy}
source /opt/ros/${ros_distro}/setup.bash
source $COLCON_WS/install/setup.bash
