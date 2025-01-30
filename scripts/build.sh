#!/bin/bash

set -e

SCRIPT_DIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $SCRIPT_DIR/utils.sh
WS_DIR=$(get_parent_dir $SCRIPT_DIR 3)

# Parse arguments
CMAKE_ARGS=""
CLEAN=false
BUILD_MICRO_ROS_SETUP=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            CMAKE_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Debug"
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --build-micro-ros-setup)
            BUILD_MICRO_ROS_SETUP=true
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--debug]"
            exit 1
            ;;
    esac
done

if [[ "$CLEAN" == "true" ]]; then
    $SCRIPT_DIR/clean_ws.sh
fi

PACKAGES_TO_SKIP=("micro_ros_agent")
if [[ "$CLEAN" != "true" ]] && [[ "$BUILD_MICRO_ROS_SETUP" != "true" ]]; then
    PACKAGES_TO_SKIP+=("micro_ros_setup")
fi

pushd $WS_DIR

echo "Installing ROS 2 dependencies"
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
echo ""

echo "Building ROS 2 packages"
colcon build --symlink-install \
        --packages-skip ${PACKAGES_TO_SKIP[@]} \
        --event-handlers console_cohesion+ \
        $CMAKE_ARGS
echo ""

source install/setup.bash
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
echo ""

echo "Creating bags directory"
mkdir -p bags

echo "Adding ROS 2 setup to bashrc"
$SCRIPT_DIR/update_bashrc.sh

popd
