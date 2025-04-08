#!/usr/bin/env bash

# Exit on any error
set -e

# Source utils
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

# Set ROS distro to jazzy if not set
ros_distro=${ROS_DISTRO:-jazzy}

# Create workspace directory
mkdir -p $ws_dir/src
pushd $ws_dir

# Download MoveIt2 source code
if [ ! -d "src/moveit2" ]; then
    print_status "Downloading MoveIt2 source code..."
    git clone https://github.com/valmikikothare/moveit2.git -b ${ros_distro} src/moveit2
else
    print_status "Pulling MoveIt2 source code..."
    pushd src/moveit2
    if ! git pull; then
        print_status "WARNING: Git pull failed for moveit2 repository, continuing..."
    fi
    popd
fi
popd

print_status "Download complete!"
