#!/bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_ws_dir=$(get_parent_dir $_script_dir 3)

_ros_distro=${ROS_DISTRO:-"jazzy"}

_commands=(
    "source /opt/ros/$_ros_distro/setup.bash"
    "source $_ws_dir/install/setup.bash"
    "PATH=$HOME/.local/bin:\$PATH"
)

for _command in "${_commands[@]}"; do
    if ! grep -Fxq "$_command" "$HOME/.bashrc"; then
        echo "$_command" >> "$HOME/.bashrc"
    fi
done
