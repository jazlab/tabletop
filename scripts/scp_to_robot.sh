#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
project_dir=$(get_parent_dir $script_dir 1)
source $project_dir/env_files/robot.env

if [ $# -eq 0 ]; then
    scp $project_dir/ursim/programs/*.urcap robot:/ursim/programs/
    source="$project_dir/ursim/programs/*.urcap"
    dest="/ursim/programs/"
elif [ $# -eq 2 ]; then
    source="$1"
    dest="$2"
else
    echo "Usage: $0 [source_dir dest_dir]"
    echo "If no arguments provided, copies *.urcap from ursim/programs to /ursim/programs/"
    exit 1
fi

scp $source $dest
