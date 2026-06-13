#!/bin/bash

set -eo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Entrypoint must be run as root"
    exit 1
fi

/fix-uid-gid.sh

exec gosu mules /user-entrypoint.sh "$@"
