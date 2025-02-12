#!/bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_project_dir=$(get_parent_dir $_script_dir 1)
source $_project_dir/env_files/robot.env

if [ $# -eq 0 ]; then
    scp $_project_dir/ursim/programs/*.urcap robot:/ursim/programs/
    _source="$_project_dir/ursim/programs/*.urcap"
    _dest="/ursim/programs/"
elif [ $# -eq 2 ]; then
    _source="$1"
    _dest="$2"
else
    echo "Usage: $0 [source_dir dest_dir]"
    echo "If no arguments provided, copies *.urcap from ursim/programs to /ursim/programs/"
    exit 1
fi

scp $_source $_dest
