#!/bin/bash

if grep "Initialization of Bluetooth controller done" /app/flicd.log; then
    if grep "A client was disconnected" /app/flicd.log; then
        echo "Client disconnected, container unhealthy."
        exit 1
    else
        echo "Server waiting or client connected, container healthy"
        exit 0
    fi
else
    echo "Server not yet ready, container unhealthy"
    exit 1
fi
