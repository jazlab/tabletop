#!/bin/bash

if (/bin/netstat -an | grep LISTEN | grep -c $PORT); then
    echo "Client still connected, container healthy."
    exit 0
else
    echo "No client connected, container unhealthy."
    exit 1
fi
