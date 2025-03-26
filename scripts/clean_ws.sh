#!/bin/bash

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --all)
            all=true
            shift
            ;;
        --micro-ros)
            micro_ros=true
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--all] [--micro-ros]"
            exit 1
            ;;
    esac
done


if [ "$all" = true ]; then
    print_status "Cleaning build, install, and log directories (including moveit)"
    sudo rm -rf $ws_dir/build $ws_dir/install
else
    print_status "Cleaning build, install, and log directories (except moveit)"
    dirs_to_clean=("tabletop*")
    if [ "$micro_ros" = true ]; then
        dirs_to_clean+=("micro_ros_agent" "micro_ros_msgs" "drive_base_msgs")
    fi
    for dir in "${dirs_to_clean[@]}"; do
        rm -rf $ws_dir/install/$dir $ws_dir/build/$dir
    done
fi
sudo rm -rf $ws_dir/log
