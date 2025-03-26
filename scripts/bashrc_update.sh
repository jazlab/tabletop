#!/bin/bash

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)


moveit_ws_dir=$HOME/moveit_ws
# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --display)
            display=$2
            shift
            shift
            ;;
        --moveit)
            moveit=true
            shift
            ;;
        --moveit-ws)
            moveit_ws_dir=$2
            shift
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--display <display>]"
            exit 1
            ;;
    esac
done

ros_distro=${ROS_DISTRO:-"jazzy"}

commands=(
    "source /opt/ros/$ros_distro/setup.bash"
    "source $ws_dir/install/setup.bash"
    "export PATH=$HOME/.local/bin:\$PATH"
)

if [ "$display" == "novnc" ]; then
    if grep -qE '^export DISPLAY' "$HOME/.bashrc"; then
        sed -i "s/^export DISPLAY.*/export DISPLAY=novnc:0.0/" "$HOME/.bashrc"
    else
        echo "export DISPLAY=novnc:0.0" >> "$HOME/.bashrc"
    fi
elif [ "$display" == "x11" ]; then
    sed -i '/^export DISPLAY/d' "$HOME/.bashrc"
elif [ -n "$display" ]; then
    echo "Error: Invalid display argument: $display"
    exit 1
fi

if [ "$moveit" == "true" ]; then
    commands+=("source $moveit_ws_dir/install/setup.bash")
fi

for command in "${commands[@]}"; do
    if ! grep -Fxq "$command" "$HOME/.bashrc"; then
        echo "$command" >> "$HOME/.bashrc"
    fi
done
