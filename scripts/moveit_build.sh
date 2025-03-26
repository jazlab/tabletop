#!/bin/bash

set -e

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh

# Take ws_dir as an argument, default to ~/moveit_ws
ws_dir=$HOME/moveit_ws
while [[ $# -gt 0 ]]; do
    case $1 in
        --ws-dir)
            ws_dir=$2
            shift
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--ws-dir <path>]"
            exit 1
            ;;
    esac
done

# Set ROS distro to jazzy if not set
ros_distro=${ROS_DISTRO:-jazzy}

pushd $ws_dir
print_status "Removing any existing MoveIt2 debian packages"
sudo apt remove -y ros-${ros_distro}-moveit* || true
print_status "Updating rosdep"
rosdep update
print_status "Installing MoveIt2 dependencies"
rosdep install --from-paths src --ignore-src --rosdistro ${ros_distro} -y
print_status "Building MoveIt2"
colcon build --symlink-install \
    --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
popd

print_status "Adding MoveIt2 setup to bashrc"
$script_dir/bashrc_update.sh --moveit
