#!/bin/bash

set -e

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

pushd $ws_dir

# Parse arguments
cmake_args=("-DUAGENT_BUILD_EXECUTABLE=OFF" "-DUAGENT_P2P_PROFILE=OFF" "--no-warn-unused-cli")
clean=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            cmake_args+=("-DCMAKE_BUILD_TYPE=Debug")
            shift
            ;;
        --clean)
            clean=true
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--debug] [--clean]"
            exit 1
            ;;
    esac
done

if [[ "$clean" == "true" ]]; then
    $script_dir/clean_ws.sh
fi

echo "Installing ROS 2 dependencies"
ros_distro=${ROS_DISTRO:-"jazzy"}
source /opt/ros/$ros_distro/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
echo ""

echo "Manually installing Python dependencies"
pip install -r src/tabletop/ros/requirements.txt

echo "Building ROS 2 packages"
colcon build --symlink-install \
        --event-handlers console_cohesion+ \
        --cmake-args ${cmake_args[@]}
echo ""

echo "Creating bags directory"
mkdir -p bags

echo "Adding ROS 2 setup to bashrc"
$script_dir/bashrc_update.sh

popd
