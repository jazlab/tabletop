#!/usr/bin/env bash

set -e

script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh
repo_dir=$(get_parent_dir $script_dir 1)

pushd $repo_dir

source venv/bin/activate
python ros/tabletop_server/tabletop_server/flic_client/piano.py "$@"

popd
