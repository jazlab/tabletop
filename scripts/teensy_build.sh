#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
project_dir=$(get_parent_dir $script_dir 1)

target_arg="--target upload"
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
        -y)
            yes=true
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
    if [ "$yes" != "true" ]; then
        read -p "Are you sure you want to clean the build directory? This will delete all build artifacts. [y/N] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Aborting clean operation"
            exit 0
        fi
    fi
    print_status "Cleaning build directory"
    sudo rm -rf $project_dir/ros/tabletop_teensy/.pio
fi

print_status "Building Teensy code"
while ! pio run $target_arg --project-dir $project_dir/ros/tabletop_teensy; do
    print_status "Build failed, retrying..."
    sleep 1
done
