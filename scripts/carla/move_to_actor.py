#!/usr/bin/python3

"""Move the spectator of the main carla window to target_role actor.

The offsets for the camera are hardcoded for a view from behind.

USAGE:
    $ python3 ./scripts/carla/move_to_actor.py ego_vehicle
"""

import os
import sys
import carla
import numpy as np


def main(target_role):
    client = carla.Client(os.environ.get("WINDOWS_HOST"), 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    world.wait_for_tick()

    found = False
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == target_role:
            found = True
            break

    if not found:
        raise LookupError(f'No actor found with `role_name` == {target_role}')

    loc = actor.get_transform().location
    rot = actor.get_transform().rotation

    transform = carla.Transform(
        carla.Location(loc.x - 7.5*np.cos(rot.yaw), loc.y, loc.z + 3),
        carla.Rotation(rot.pitch - 15, rot.yaw, rot.roll)
    )

    world.get_spectator().set_transform(transform)


if __name__ == "__main__":
    target_role = 'ego_vehicle' if len(sys.argv) == 1 else sys.argv[1]
    main(target_role)
