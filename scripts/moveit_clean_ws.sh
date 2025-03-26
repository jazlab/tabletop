#!/bin/bash

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh

# Take ws_dir as an argument, default to ~/moveit_ws
ws_dir=$HOME/moveit_ws
while [[ $# -gt 0 ]]; do
    case $1 in
        --ws-dir)
            ws_dir=$2
            shift
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--ws-dir <path>]"
            exit 1
            ;;
    esac
done

print_status "Cleaning build, install, and log directories"
sudo rm -rf $ws_dir/build $ws_dir/install $ws_dir/log
