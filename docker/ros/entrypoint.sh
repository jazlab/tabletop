#!/bin/bash

set -eo pipefail

# setup project workspace or ros2 environment
if [[ -f /tabletop/setup.bash ]]; then
    source /tabletop/setup.bash
else
    source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi

exec "$@"
