#!/bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_project_dir=$(get_parent_dir $_script_dir 1)

_clean=false
target_arg="--target upload"
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            _clean=true
            shift
            ;;
        --no-upload)
            target_arg=""
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--clean] [--no-upload]"
            exit 1
            ;;
    esac
done

if [[ "$_clean" == "true" ]]; then
    sudo rm -rf $_project_dir/ros/tabletop_teensy/.pio
fi

pio run $target_arg --project-dir $_project_dir/ros/tabletop_teensy