#!/bin/bash

# Exit on any error
set -e

# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

pushd $ws_dir

# Update system packages and rosdep
print_status "Updating system packages and rosdep..."
sudo apt-get update
sudo apt-get dist-upgrade -y
rosdep update

# Set ROS distro to jazzy if not set
ROS_DISTRO=${ROS_DISTRO:-jazzy}

# Download MoveIt2 source code
print_status "Downloading MoveIt2 source code..."
if [ ! -d "src/moveit2" ]; then
    git clone https://github.com/valmikikothare/moveit2.git -b $ROS_DISTRO src/moveit2
else
    pushd src/moveit2
    if ! git pull; then
        echo "WARNING: Git pull failed for moveit2 repository, continuing..."
    fi
    popd
fi

# Download MoveIt2 dependency repos
pushd src/moveit2
for repo in moveit2.repos $(f="moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do
    vcs import < "$repo"
done
popd

print_status "Download complete!"

popd
