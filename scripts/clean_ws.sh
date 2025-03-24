#!/bin/bash

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh
ws_dir=$(get_parent_dir $script_dir 3)

# Default to not cleaning moveit
clean_moveit=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --moveit)
      clean_moveit=true
      shift
      ;;
    *)
      shift
      ;;
  esac
done

# Clean MoveIt artifacts
if [ "$clean_moveit" = true ]; then
  print_status "Cleaning build, install, and log directories (including MoveIt artifacts)"
  sudo rm -rf $ws_dir/build $ws_dir/install
else
  print_status "Cleaning build, install, and log directories (excluding MoveIt artifacts)"
  # Find and remove all directories except those related to moveit
  find $ws_dir/build -mindepth 1 -maxdepth 1 -not -name "moveit" -exec sudo rm -rf {} \;
  find $ws_dir/install -mindepth 1 -maxdepth 1 -not -name "moveit" -exec sudo rm -rf {} \;
fi
sudo rm -rf $ws_dir/log
