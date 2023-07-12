# Carla-ROS-Framework

## Installation

Within your [WLS](https://learn.microsoft.com/en-us/windows/wsl/setup/environment) command-line interface, clone the [repository](https://bitbucket.efs-techhub.com/scm/at01447/carla_simulation_pipeline.git) and open a remove session with [Visual Studio Code](https://code.visualstudio.com/):

```sh
cd ~/
git clone -b experimental/toni https://bitbucket.efs-techhub.com/scm/at01447/carla_simulation_pipeline.git carla_ros_framework

cd carla_ros_framework
code .
```

[Visual Studio Code](https://code.visualstudio.com/) now establishes a remote session to this directory. Here, you will be asked to `Reopen in Container` in the bottom right corner. If use miss it, press `CTRL+SHIFT+P` and search for it, to evoke the command yourself.

> This environment now contains [ROS2 Foxy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html) and [ROS/ROS2 bridge for CARLA simulator](https://github.com/carla-simulator/ros-bridge) to work with.

Within **Windows**, you need to install [Carla](https://github.com/carla-simulator/carla/releases) by downloading [CARLA_0.9.14.zip](https://carla-releases.s3.eu-west-3.amazonaws.com/Windows/CARLA_0.9.14.zip) and unpacking it into your *favorite* directory.

## How to

This section contains all the steps to get an *exemplary* simulation running.

- [x] Start `CarlaUE4.exe` within **Windows**

    You can specify the *resolution* and *quality* if needed with `-ResX=200 -ResY=100 --quality-level=Low`

- [x] Start all necessary tasks from the [ROS/ROS2 bridge for CARLA simulator](https://github.com/carla-simulator/ros-bridge) 

    With the building of the **devcontainer**, a plugin called [Task Buttons](https://marketplace.visualstudio.com/items?itemName=spencerwmiles.vscode-task-buttons) was installed. This plugin adds buttons for configurable tasks to the **status bar** at the bottom of [Visual Studio Code](https://code.visualstudio.com/).

    Here, you have to start `Carla-ROS-Bridge`, `Set-Up-Scenario` and `Control-Interface`. A detailed description for these task can be found further down below.
