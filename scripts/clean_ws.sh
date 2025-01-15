#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f $0))
source $SCRIPT_DIR/env.sh


# Default to 4 levels up if not specified
LEVELS_UP=${1:-3}
UP_PATH=$(printf '../%.0s' $(seq 1 $LEVELS_UP))
WS_DIR=$(cd $(dirname $(readlink -f $0))/$UP_PATH && pwd)

echo "Cleaning workspace: $WS_DIR"

rm -rf $WS_DIR/build $WS_DIR/install $WS_DIR/log
