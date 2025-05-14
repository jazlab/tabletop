#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
project_dir=$(get_parent_dir $script_dir 1)
source $project_dir/env_files/robot.env

print_status "Generating SSH key"
ssh-keygen
print_status "Copying SSH key to robot (password: jazlab)"
ssh-copy-id "root@$ROBOT_IP"

# Check if .ssh/config exists, create if not
if [ ! -f ~/.ssh/config ]; then
    mkdir -p ~/.ssh
    touch ~/.ssh/config
    chmod 600 ~/.ssh/config
fi

# Check if entry exists with current ROBOT_IP
if ! grep -q "HostName $ROBOT_IP" ~/.ssh/config; then
    # Add new entry
    echo -e "Host robot\n  HostName $ROBOT_IP\n  User root\n" >> ~/.ssh/config
    print_status "Added new SSH config entry for robot at $ROBOT_IP"
else
    print_status "SSH config entry for $ROBOT_IP already exists"
fi
