#!/bin/bash

set -eo pipefail

if [ "$RUN_XTERM" != "true" ]; then
    rm -f /app/conf.d/xterm.conf
fi

cp /app/conf.wm/$WINDOW_MANAGER.conf /app/conf.d/

exec supervisord -c /app/supervisord.conf
