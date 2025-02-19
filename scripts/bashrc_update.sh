#!/bin/bash

# Parse arguments
display_novnc=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --display_novnc)
            display_novnc=true
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--display_novnc]"
            exit 1
            ;;
    esac
done

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

ros_distro=${ROS_DISTRO:-"jazzy"}

commands=(
    "source /opt/ros/$ros_distro/setup.bash"
    "source $ws_dir/install/setup.bash"
    "export PATH=$HOME/.local/bin:\$PATH"
)

if [[ "$display_novnc" == "true" ]]; then
    commands+=("export DISPLAY=novnc:0.0")
fi

for command in "${commands[@]}"; do
    if ! grep -Fxq "$command" "$HOME/.bashrc"; then
        echo "$command" >> "$HOME/.bashrc"
    fi
done

