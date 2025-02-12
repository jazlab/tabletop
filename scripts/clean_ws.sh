#!/bin/bash

_script_dir=$(dirname $(readlink -f $0))
source $_script_dir/utils.sh
_ws_dir=$(get_parent_dir $_script_dir 3)

echo "Cleaning workspace: $_ws_dir"

sudo rm -rf $_ws_dir/build $_ws_dir/install $_ws_dir/log
