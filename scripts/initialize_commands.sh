#!/usr/bin/env sh

# Script for .devcontainer
#   "initializeCommand": "scripts/initialize_commands.sh"

CYAN='\033[1;36m'
RESET='\e[0m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
PURPLE='\033[0;35m'

if [ $(uname -a | grep microsoft | wc -l) -ge 1 ]; then
    echo "${CYAN}WSL${RESET}    ${RED}$(uname -a | cut -d ' ' -f 3)${RESET}"

    SYSTEM_NAME=$(lsb_release -d | cut -d ':' -f2 | xargs)
    IP_ADDRESS=$(ipconfig.exe | grep 'vEthernet (WSL)' -A4 | cut -d ":" -f 2 | tail -n1 | sed -e 's/\s*//g')

else
    SYSTEM_NAME=$(lsb_release -d | cut -d ':' -f 2 | xargs)
    IP_ADDRESS=$(hostname -I | cut -d ' ' -f 1)
fi

echo "HOST_IP_ADDRESS=$IP_ADDRESS" > ./.devcontainer/env

echo "${CYAN}HOST${RESET}   $(hostname) [${PURPLE}$SYSTEM_NAME${RESET}]"
echo "${CYAN}ENV${RESET}    HOST_IP_ADDRESS${RESET}=${PURPLE}$IP_ADDRESS${RESET}"