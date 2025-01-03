#!/bin/bash

set -e

if [ -z "$1" ] || [ "$1" != "debug" ]; then
    CMAKE_ARGS=()
else
    CMAKE_ARGS=("--cmake-args" "-DCMAKE_BUILD_TYPE=Debug")
fi

pushd $HOME/ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -i -y
colcon build --symlink-install \
        --event-handlers console_cohesion+ \
        --base-paths /root/ws \
        "${CMAKE_ARGS[@]}"
mkdir -p bags
source $HOME/ws/install/setup.bash

if ! grep -Fxq "source $HOME/ws/install/setup.bash" $HOME/.bashrc
then
    echo "source $HOME/ws/install/setup.bash" >> $HOME/.bashrc
fi
popd