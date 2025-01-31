#! /bin/bash

SCRIPT_DIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $SCRIPT_DIR/utils.sh
PROJECT_DIR=$(get_parent_dir $SCRIPT_DIR 1)

# Create a symlink to tabletop_msgs from the tabletop_teensy/extra_packages directory
ln -s $PROJECT_DIR/ros/tabletop_msgs $PROJECT_DIR/ros/tabletop_teensy/extra_packages/tabletop_msgs
