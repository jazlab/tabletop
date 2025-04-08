#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
project_dir=$(get_parent_dir $script_dir 1)

target_arg="--target upload"
clean=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            clean=true
            shift
            ;;
        --no-upload)
            unset target_arg
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--clean] [--no-upload]"
            exit 1
            ;;
    esac
done

if [ "$clean" = "true" ]; then
    sudo rm -rf $project_dir/ros/tabletop_teensy/.pio
fi

pio run $target_arg --project-dir $project_dir/ros/tabletop_teensy
