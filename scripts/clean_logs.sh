#!/usr/bin/env bash

set -e

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh
repo_dir=$(get_parent_dir $script_dir 1)

# Parse arguments
skip_confirmation=false
while [[ $# -gt 0 ]]; do
  case $1 in
    -y|--yes)
      skip_confirmation=true
      shift
      ;;
    *)
      shift
      ;;
  esac
done

# Get the size of the logs directory
if [ -d "$ROS_LOG_DIR" ]; then
    dir_size=$(du -sh "$ROS_LOG_DIR" | cut -f1)
    print_status "Logs directory size: $dir_size"

    # Count number of files/directories in logs_dir
    item_count=$(ls -1 "$ROS_LOG_DIR" | wc -l)
    print_status "Number of items in logs directory: $item_count"
else
    print_status "Logs directory does not exist: $ROS_LOG_DIR"
    exit 1
fi

# Ask for confirmation before deletion
if [ "$skip_confirmation" = "false" ]; then
    read -p "Do you want to proceed with deleting all logs in $ROS_LOG_DIR? (y/n): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        print_status "Operation cancelled"
        exit 0
    fi
fi

# Remove all bag files in the bags directory
print_status "Removing all logs in $ROS_LOG_DIR"
rm -rf $ROS_LOG_DIR/*
print_status "Cleanup complete"
