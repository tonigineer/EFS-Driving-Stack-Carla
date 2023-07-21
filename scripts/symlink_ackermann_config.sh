#!/usr/bin/env sh

# NOTE: Parameter file for Ackermann Control of
# ROS-Bridge (default K* result in really poor behavior)
rm ~/ros2_ws/src/carla_ros_bridge/carla_ackermann_control/config/settings.yaml
ln -s /$WORKSPACE_DIR/configs/ackermann_control/settings.yaml ~/ros2_ws/src/carla_ros_bridge/carla_ackermann_control/config/settings.yaml

# https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux

CYAN='\033[1;36m'
RESET='\e[0m'
YELLOW='\033[0;33m'

echo "${CYAN}settings.yaml${RESET} of ROS2-Package ${YELLOW}carla_ackermann_control${RESET} was symlinked to ${CYAN}configs/ackermann_control/settings.yaml${RESET}"

echo -n "Due to new config, building ${YELLOW}~/ros2_ws/${RESET} ... "
cd ~/ros2_ws && colcon build > /dev/null
echo "done."
