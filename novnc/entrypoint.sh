#!/bin/bash

set -e

case "$(echo "$RUN_XTERM" | tr '[:upper:]' '[:lower:]')" in
  false|no|n|0|'')
    rm -f /app/conf.d/xterm.conf
    ;;
esac

cp /app/conf.wm/$(echo "$WINDOW_MANAGER" | tr '[:upper:]' '[:lower:]').conf /app/conf.d/

exec supervisord -c /app/supervisord.conf
