#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
project_dir=$(get_parent_dir $script_dir 1)
# source $project_dir/env_files/robot.env

# if [ $# -eq 0 ]; then
#     source="$project_dir/ur_robot/programs/*.urcap"
#     dest="root@$ROBOT_IP:/programs"
# elif [ $# -eq 2 ]; then
#     source="$1"
#     dest="root@$ROBOT_IP:$2"
# else
#     print_status "Usage: $0 [source_dir dest_dir]"
#     print_status "If no arguments provided, copies *.urcap from ur_robot/programs to /ursim/programs/"
#     exit 1
# fi

if [ $# -ne 1 ]; then
    print_status "Usage: $0 <robot_ip>"
    exit 1
fi

source="$project_dir/ur_robot/programs/*.urcap"
dest="root@$1:/programs"
scp $source $dest
