#!/bin/bash

ROOT=$(dirname $(dirname $(readlink -f $0)))
source $ROOT/env_files/robot.env

if [ $# -eq 0 ]; then
    scp ursim/programs/*.urcap robot:/ursim/programs/
    SOURCE="ursim/programs/*.urcap"
    DEST="/ursim/programs/"
elif [ $# -eq 2 ]; then
    SOURCE="$1"
    DEST="$2"
else
    echo "Usage: $0 [source_dir dest_dir]"
    echo "If no arguments provided, copies *.urcap from ursim/programs to /ursim/programs/"
    exit 1
fi

scp $SOURCE $DEST