#!/bin/bash

set -e

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_ws_dir=$(get_parent_dir $_script_dir 3)

# Parse arguments
_cmake_args=""
_clean=false
_build_micro_ros_setup=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            _cmake_args="--cmake-args -DCMAKE_BUILD_TYPE=Debug"
            shift
            ;;
        --clean)
            _clean=true
            shift
            ;;
        --build-micro-ros-setup)
            _build_micro_ros_setup=true
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--debug]"
            exit 1
            ;;
    esac
done

if [[ "$_clean" == "true" ]]; then
    $_script_dir/clean_ws.sh
fi

_packages_to_skip=("micro_ros_agent")
if [[ "$_clean" != "true" ]] && [[ "$_build_micro_ros_setup" != "true" ]]; then
    _packages_to_skip+=("micro_ros_setup")
fi

pushd $_ws_dir

echo "Installing ROS 2 dependencies"
_ros_distro=${ROS_DISTRO:-"jazzy"}
source /opt/ros/$_ros_distro/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
echo ""

echo "Building ROS 2 packages"
colcon build --symlink-install \
        --packages-skip ${_packages_to_skip[@]} \
        --event-handlers console_cohesion+ \
        $_cmake_args
echo ""

source /opt/ros/$_ros_distro/setup.bash
source install/setup.bash
ros2 run micro_ros_setup build_agent.sh
echo ""

echo "Creating bags directory"
mkdir -p bags

echo "Adding ROS 2 setup to bashrc"
$_script_dir/update_bashrc.sh

source /opt/ros/$_ros_distro/setup.bash
source install/setup.bash

popd
