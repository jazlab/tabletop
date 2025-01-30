#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f $0))
source $SCRIPT_DIR/utils.sh
WS_DIR=$(get_parent_dir $SCRIPT_DIR 3)

echo "Cleaning workspace: $WS_DIR"

sudo rm -rf $WS_DIR/build $WS_DIR/install $WS_DIR/log
