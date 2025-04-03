#!/usr/bin/env bash

script_dir=$(dirname $(readlink -f $0))
source $script_dir/utils.sh

flic_client_exec=$(get_parent_dir $script_dir 1)/flic_client/simpleclient

$flic_client_exec 172.17.0.1 5551
