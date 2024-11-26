#!/bin/bash

pushd $HOME/ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -i -y
colcon build --packages-select tabletop_msgs tabletop_server tabletop_moveit_config tabletop_moveit_interface
source $HOME/ws/install/setup.bash

if ! grep -Fxq "source $HOME/ws/install/setup.bash" $HOME/.bashrc
then
    echo "source $HOME/ws/install/setup.bash" >> $HOME/.bashrc
fi
popd