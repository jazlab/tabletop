#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh
repo_dir=$(get_parent_dir $script_dir 1)

sudo chown -R $USER:$USER $repo_dir
