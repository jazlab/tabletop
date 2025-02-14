#!/bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_project_dir=$(get_parent_dir $_script_dir 1)

# Create a symlink to tabletop_msgs from the tabletop_teensy/extra_packages directory
ln -s $_project_dir/ros/tabletop_msgs $_project_dir/ros/tabletop_teensy/extra_packages/tabletop_msgs
