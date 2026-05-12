#!/bin/bash

set -eo pipefail

# Setup project workspace or ROS2 environment
if [ -s /tabletop/setup.bash ]; then
    source /tabletop/setup.bash
else
    source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi

# Execute provided command
exec "$@"
