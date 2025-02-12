#! /bin/bash

_script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $_script_dir/utils.sh
_project_dir=$(get_parent_dir $_script_dir 1)

sudo chmod +x $_script_dir/*.sh
sudo chmod +x $_project_dir/ur_robot/entrypoint.sh
