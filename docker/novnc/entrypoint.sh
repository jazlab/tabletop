#!/bin/bash

set -eo pipefail

security_dir=/run/tabletop-novnc
runtime_dir=/run/tabletop-novnc-runtime

for required_file in credentials server.crt server.key Xauthority; do
    if [ ! -r "$security_dir/$required_file" ]; then
        echo "Missing noVNC security file: $security_dir/$required_file" >&2
        echo "Run 'tt-novnc-security init' on the host, then recreate noVNC." >&2
        exit 1
    fi
done

credentials=$(head -n 1 "$security_dir/credentials")
username=${credentials%%:*}
password=${credentials#*:}
if [ -z "$username" ] || [ -z "$password" ] || [ "$credentials" = "$password" ]; then
    echo "Invalid noVNC credentials file; expected username:password." >&2
    exit 1
fi

umask 077
mkdir -p "$runtime_dir"
x11vnc -storepasswd "$password" "$runtime_dir/rfb.pass" >/dev/null
export PYTHONPATH="/app${PYTHONPATH:+:$PYTHONPATH}"
export XAUTHORITY="$security_dir/Xauthority"
mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix

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
