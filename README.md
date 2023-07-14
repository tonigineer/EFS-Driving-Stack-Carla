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

- [x] Start all necessary tasks from the [ROS/ROS2 bridge for CARLA simulator](https://github.com/carla-simulator/ros-bridge) and this [Repository](README.md)

    With the building of the [devcontainer](.devcontainer/devcontainer.json), a plugin called [Task Buttons](https://marketplace.visualstudio.com/items?itemName=spencerwmiles.vscode-task-buttons) was installed. This plugin adds buttons for configurable tasks to the **status bar** at the bottom of [Visual Studio Code](https://code.visualstudio.com/).

    1. `Carla-ROS-Bridge` - Kinda selfexplanatory - but in case, [there](https://github.com/carla-simulator/ros-bridge/blob/master/README.md) you go
    2. `Set-Up-Scenario` - Place all Actors with Sensory according to selected [configuration](configs/scenarios/follow_route.json)
    3. `Goto ego` - Move the [spectator](https://carla.readthedocs.io/en/latest/tuto_G_getting_started/#the-spectator) (main Carla window) to the [actor](https://carla.readthedocs.io/en/latest/core_actors/) with the role name `ego_vehicle`
    4. `Dashboard` - Start a dashboard, that uses RGB-Cameras around the vehicle to show different angles
    5. `Start planner` - Start the [planner](src/planner/planner/planner.py) to generate a random [route](https://github.com/carla-simulator/ros-bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py) and publish a [reference]() for the controller
