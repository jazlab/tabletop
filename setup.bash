# Get bin directory and source utils
export TABLETOP_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
source $TABLETOP_DIR/utils.bash

# Set environment variables
export COLCON_WS=$(get_parent_dir $TABLETOP_DIR 2)
export ROS_LOG_DIR=$TABLETOP_DIR/logs
export TABLETOP_BAG_DIR=$TABLETOP_DIR/results/bags
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SIM_ROBOT_IP=192.168.12.20
export SIM_REVERSE_IP=192.168.12.10
export ROBOT_IP=192.168.13.20
export REVERSE_IP=192.168.13.10
export PYTHONUNBUFFERED=1

# Update PATH
export PATH=$TABLETOP_DIR/bin:$PATH

# Aliases
alias tt_server="ros2 launch tabletop_server server.launch.py"
alias tt_commander="ros2 launch tabletop_server commander.launch.py"
alias tt_tasks="ros2 launch tabletop_tasks tasks.launch.py"
alias tree="tree -I 'build|install|logs|results'"
