#!/bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_project_dir=$(get_parent_dir $_script_dir 1)

_clean=false
while [[ $# -gt 0 ]]; do
    case $1 in

        --clean)
            _clean=true
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--clean]"
            exit 1
            ;;
    esac
done

if [[ "$_clean" == "true" ]]; then
    rm -rf $_project_dir/ros/tabletop_teensy/.pio
fi

pio run --target upload --project-dir $_project_dir/ros/tabletop_teensy
