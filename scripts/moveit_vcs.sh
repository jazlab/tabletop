#!/usr/bin/env bash

# Exit on any error
set -e

# Source utils
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

# Set ROS distro to jazzy if not set
ros_distro=${ROS_DISTRO:-jazzy}

# Download MoveIt2 dependency repos
pushd $ws_dir/src/moveit2
for repo in moveit2.repos $(f="moveit2_${ros_distro}.repos"; test -r $f && echo $f); do
    print_status "Importing $repo..."
    vcs import < "$repo"
done
popd

print_status "VCS import complete!"
