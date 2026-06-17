#!/usr/bin/env bash
# Copy UR robot program files to robot via SCP
# Usage: scp-urcaps.sh
# Wraps: scp
# Runs on: host or container (accessible from both via PATH)
# Environment: TABLETOP_DIR, LEFT_ROBOT_IP, RIGHT_ROBOT_IP
# Notes: Requires SSH access to robot as root; copies .urcap files from ur_robot/programs/

set -e

bin_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
source $bin_dir/../../setup.bash
source $bin_dir/../../bin/utils.sh

source="$TABLETOP_DIR/ur_robot/programs/*.urcap"
left_dest="root@$LEFT_ROBOT_IP:/programs"
right_dest="root@$RIGHT_ROBOT_IP:/programs"

error=false
if scp "$source" "$left_dest"; then
    print_status "Successfully copied $source to left robot ($left_dest)"
else
    print_error "Could not copy $source to left robot ($left_dest)"
    error=true
fi
if scp "$source" "$right_dest"; then
    print_status "Successfully copied $source to right robot ($right_dest)"
else
    print_error "Could not copy $source to right robot ($right_dest)"
    error=true
fi

if [ "$error" = true ]; then
    exit 1
fi
