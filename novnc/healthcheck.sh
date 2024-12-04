#!/bin/bash

# Run supervisorctl and check if any process (except print_url) is not RUNNING
if supervisorctl -c /app/supervisord.conf status all | grep -v "print_url" | awk '{print $2}' | grep -qv RUNNING; then
    echo "Not all required processes are running"
    exit 1
else
    echo "All required processes are running"
    exit 0
fi