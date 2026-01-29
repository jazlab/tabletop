#!/bin/bash

if grep "disconnected" /app/flicd.log; then
    echo "Client disconnected, container unhealthy."
    exit 1
else
    echo "Server waiting or client connected, container healthy"
    exit 0
fi
