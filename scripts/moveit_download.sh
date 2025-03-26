#!/bin/bash

# Exit on any error
set -e

# Source utils
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

# # Take ws_dir as an argument, default to ~/moveit_ws
# ws_dir=$HOME/moveit_ws
# while [[ $# -gt 0 ]]; do
#     case $1 in
#         --ws-dir)
#             ws_dir=$2
#             shift
#             shift
#             ;;
#         *)
#             echo "Error: Unknown argument $1"
#             echo "Usage: $0 [--ws-dir <path>]"
#             exit 1
#             ;;
#     esac
# done

# Set ROS distro to jazzy if not set
ros_distro=${ROS_DISTRO:-jazzy}

# Update system packages and rosdep
print_status "Updating system packages and rosdep..."
sudo apt-get update
sudo apt-get dist-upgrade -y
rosdep update

# Create workspace directory
mkdir -p $ws_dir/src
pushd $ws_dir

# Download MoveIt2 source code
if [ ! -d "src/moveit2" ]; then
    print_status "Downloading MoveIt2 source code..."
    git clone https://github.com/valmikikothare/moveit2.git -b ${ros_distro} src/moveit2
else
    print_status "Pulling MoveIt2 source code..."
    pushd src/moveit2
    if ! git pull; then
        print_status "WARNING: Git pull failed for moveit2 repository, continuing..."
    fi
    popd
fi

# Download MoveIt2 dependency repos
pushd src/moveit2
for repo in moveit2.repos $(f="moveit2_${ros_distro}.repos"; test -r $f && echo $f); do
    print_status "Importing $repo..."
    vcs import < "$repo"
done
popd

print_status "Download complete!"

popd
