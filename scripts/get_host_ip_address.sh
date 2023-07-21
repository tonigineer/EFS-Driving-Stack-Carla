#!/usr/bin/env sh

if ! command -v ipconfig.exe &> /dev/null
then
    # Linux
    IP_ADDRESS=$(hostname -I | cut -d' ' -f1)
else
    # Windows - WSL
    IP_ADDRESS=$(ipconfig.exe | grep 'vEthernet (WSL)' -A4 | cut -d":" -f 2 | tail -n1 | sed -e 's/\s*//g')
fi


echo "HOST_IP_ADDRESS=$IP_ADDRESS" > ./.devcontainer/env

CYAN='\033[1;36m'
RESET='\e[0m'
YELLOW='\033[0;33m'

echo "${CYAN}HOST_IP_ADDRESS${RESET}=${YELLOW}$IP_ADDRESS${RESET} was set for Devcontainer"