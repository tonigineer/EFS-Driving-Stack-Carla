#!/usr/bin/env sh

# Script for .devcontainer
#    "postStartCommand": "scripts/post_start_commands.sh"

CYAN='\033[1;36m'
RESET='\e[0m'
YELLOW='\033[0;33m'

echo -n "${CYAN}colcon build --symlink-install${RESET} ... "
cd $WORKSPACE_DIR && colcon build --symlink-install  > /dev/null
echo "done."
