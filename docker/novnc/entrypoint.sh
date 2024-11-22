#!/bin/bash
set -e

case $RUN_XTERM in
  false|no|n|0|'')
    rm -f /app/conf.d/xterm.conf
    ;;
esac

for wm in dwm i3 xfce fluxbox ; do
  if [[ "$WINDOW_MANAGER" != "$wm" ]]; then
    rm -f /app/conf.d/${wm}.conf
  fi
done

exec supervisord -c /app/supervisord.conf
