#!/bin/bash

set -e

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

pushd $ws_dir

# Parse arguments
debug=false
build_moveit=false
clean=false
clean_moveit=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            debug=true
            shift
            ;;
        --build-moveit)
            build_moveit=true
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

# Set CMake arguments
cmake_args=("-DUAGENT_BUILD_EXECUTABLE=OFF" "-DUAGENT_P2P_PROFILE=OFF" "--no-warn-unused-cli")
if [ "$debug" = "true" ]; then
    cmake_args+=("-DCMAKE_BUILD_TYPE=Debug")
else
    cmake_args+=("-DCMAKE_BUILD_TYPE=Release")
fi
echo "CMake args: ${cmake_args[@]}"

# Set build paths
build_paths=("$ws_dir/src/tabletop")
if [ "$build_moveit" = "true" ]; then
    build_paths+=("$ws_dir/src/moveit2")
fi
echo "Build paths: ${build_paths[@]}"

# Clean workspace
if [ "$clean" = "true" ]; then
    if [ "$clean_moveit" = "true" ]; then
        $script_dir/clean_ws.sh --moveit
    else
        $script_dir/clean_ws.sh
    fi
fi

# Remove any existing MoveIt2 debian packages
if [ "$build_moveit" = "true" ]; then
    print_status "Removing any existing MoveIt2 debian packages..."
    sudo apt remove -y ros-$ROS_DISTRO-moveit* || true
fi


# Install ROS 2 dependencies
print_status "Installing ROS 2 dependencies"
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Install Python dependencies
print_status "Installing extra Python dependencies using pip"
pip install -r src/tabletop/ros/requirements.txt

# Build ROS 2 packages
print_status "Building ROS 2 packages"
colcon build \
    --symlink-install \
    --event-handlers console_cohesion+ \
    --base-paths "${build_paths[@]}" \
    --cmake-args "${cmake_args[@]}"

print_status "Creating bags directory"
mkdir -p bags

print_status "Adding ROS 2 setup to bashrc"
$script_dir/bashrc_update.sh

popd
