#!/usr/bin/env bash

set -e

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

pushd $ws_dir

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            debug=true
            shift
            ;;
        --clean)
            clean=true
            shift
            ;;
        --clean-moveit)
            clean_moveit=true
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--debug] [--clean] [--clean-moveit]"
            exit 1
            ;;
    esac
done


# Set build paths
# build_paths=("$ws_dir/src/tabletop")

# Clean workspace
if [ "$clean_moveit" = "true" ]; then
    $script_dir/clean_ws.sh --moveit
elif [ "$clean" = "true" ]; then
    $script_dir/clean_ws.sh
fi

# Upgrade apt packages
print_status "Upgrading apt packages"
sudo apt update
sudo apt upgrade -y
# sudo apt dist-upgrade -y

# Set ROS distro to jazzy if not set
ros_distro=${ROS_DISTRO:-jazzy}

# Remove any existing MoveIt2 debian packages
# print_status "Removing any existing MoveIt2 debian packages"
# sudo apt remove -y ros-${ros_distro}-moveit* || true

# VCS import moveit2 repos
$script_dir/moveit_vcs.sh

# Install ROS 2 dependencies
print_status "Installing ROS 2 dependencies"
source /opt/ros/${ros_distro}/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ros_distro} -y

# Install Python dependencies
print_status "Installing extra Python dependencies using pip"
pip install -r src/tabletop/ros/requirements.txt

# Set CMake arguments
cmake_args=("-DUAGENT_BUILD_EXECUTABLE=OFF" "-DUAGENT_P2P_PROFILE=OFF" "--no-warn-unused-cli")
if [ "$debug" = "true" ]; then
    cmake_args+=("-DCMAKE_BUILD_TYPE=Debug")
else
    cmake_args+=("-DCMAKE_BUILD_TYPE=Release")
fi

# Build ROS 2 packages
print_status "Building ROS 2 packages"
export MAKEFLAGS="-j2"
colcon build \
    --symlink-install \
    --event-handlers console_cohesion+ \
    --cmake-args "${cmake_args[@]}" \
    --parallel-workers 2 # \
    # --base-paths "${build_paths[@]}"

print_status "Creating bags directory"
mkdir -p bags

print_status "Adding ROS 2 setup to bashrc"
$script_dir/bashrc_update.sh

popd
