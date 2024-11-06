#!/bin/bash

root=$HOME/ws/

cd $root
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --packages-select tabletop

if ! grep -Fxq "source $HOME/ws/install/setup.bash" $HOME/.bashrc
then
    echo "source $HOME/ws/install/setup.bash" $HOME/.bashrc
fi