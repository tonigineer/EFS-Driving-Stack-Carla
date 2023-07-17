VAR=$(ipconfig.exe | grep 'vEthernet (WSL)' -A4 | cut -d":" -f 2 | tail -n1 | sed -e 's/\s*//g')
echo "WINDOWS_HOST=$VAR" > ./.devcontainer/env

CYAN='\033[1;36m'
RESET='\e[0m'

echo "${CYAN}WINDOWS_HOST${RESET}=$VAR was set for Devcontainer"