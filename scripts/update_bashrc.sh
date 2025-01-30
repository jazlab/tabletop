#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $SCRIPT_DIR/utils.sh
WS_DIR=$(get_parent_dir $SCRIPT_DIR 3)

_commands=(
    "source /opt/ros/$ROS_DISTRO/setup.bash"
    "source $WS_DIR/install/setup.bash"
    "PATH=\$PATH:/root/.platformio/penv/bin"
)

for _command in "${_commands[@]}"; do
    if ! grep -Fxq "$_command" "$HOME/.bashrc"; then
        echo "$_command" >> "$HOME/.bashrc"
    fi
done
