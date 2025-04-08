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

bags_dir=$repo_dir/ros/bags

# Get the size of the bags directory
if [ -d "$bags_dir" ]; then
    dir_size=$(du -sh "$bags_dir" | cut -f1)
    print_status "Bags directory size: $dir_size"

    # Count number of files/directories in bags_dir
    item_count=$(ls -1 "$bags_dir" | wc -l)
    print_status "Number of items in bags directory: $item_count"
else
    print_status "Bags directory does not exist: $bags_dir"
    exit 1
fi

# Ask for confirmation before deletion
if [ "$skip_confirmation" = "false" ]; then
    read -p "Do you want to proceed with deleting all bag files in $bags_dir? (y/n): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        print_status "Operation cancelled"
        exit 0
    fi
fi

# Remove all bag files in the bags directory
print_status "Removing all bag files in $bags_dir"
rm -rf $bags_dir/*
print_status "Cleanup complete"
