#!/bin/bash

set -e

SCRIPT_DIR=$(dirname $(readlink -f $0))
source $SCRIPT_DIR/utils.sh
WS_DIR=$(get_parent_dir $SCRIPT_DIR 3)

# Parse arguments
CMAKE_ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            CMAKE_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=Debug"
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--debug]"
            exit 1
            ;;
    esac
done


pushd $WS_DIR
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -i -y
colcon build --symlink-install \
        --event-handlers console_cohesion+ \
        --base-paths /root/ws \
        "$CMAKE_ARGS"
mkdir -p bags

if ! grep -Fxq "source $WS_DIR/install/setup.bash" $HOME/.bashrc
then
    echo "source $WS_DIR/install/setup.bash" >> $HOME/.bashrc
fi
popd