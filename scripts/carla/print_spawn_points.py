#!/usr/bin/python3

import os
import carla


def main():
    host = os.environ.get("WINDOWS_HOST")
    client = carla.Client(host, 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()

    for spawn_point in spawn_points:
        print(spawn_point)


if __name__ == "__main__":
    main()
