#!/bin/bash

set -eo pipefail

# Add nvm, npm, node, and any globally installed npm packages to path
if [ -d "$HOME/.nvm" ]; then
    export NVM_DIR="$HOME/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
fi

# Add ~/.local/bin and ~/bin to PATH
[ -d "$HOME/.local/bin" ] && export PATH="$HOME/.local/bin:$PATH"
[ -d "$HOME/bin" ] && export PATH="$HOME/bin:$PATH"

# Setup project workspace or ROS2 environment
if [ -f /tabletop/setup.bash ]; then
    source /tabletop/setup.bash
else
    source "/opt/ros/$ROS_DISTRO/setup.bash" --
fi

# Execute provided command
exec "$@"
