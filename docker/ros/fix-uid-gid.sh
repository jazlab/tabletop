#!/bin/bash

set -eo pipefail

changed=false
if [ -n "$HOST_GID" ] && [ "$HOST_GID" -ne "$(id mules -g)" ]; then
    groupmod -o --gid "$HOST_GID" mules
    changed=true
fi
if [ -n "$HOST_UID" ] && [ "$HOST_UID" -ne "$(id mules -u)" ]; then
    usermod -o --uid "$HOST_UID" mules
    changed=true
fi

if [ "$changed" = true ]; then
    chown -R mules:mules /home/mules
fi
