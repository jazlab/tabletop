#!/bin/bash

set -eo pipefail

if [ "$RUN_XTERM" != "true" ]; then
    rm -f /app/conf.d/xterm.conf
fi

cp /app/conf.wm/$WINDOW_MANAGER.conf /app/conf.d/

temp_value=${DISPLAY#*:}
export DISPLAY_NUMBER=${temp_value%.*}
export DISPLAY_SCREEN=${temp_value#*.}

echo $DISPLAY_NUMBER
echo $DISPLAY_SCREEN

exec supervisord -c /app/supervisord.conf
