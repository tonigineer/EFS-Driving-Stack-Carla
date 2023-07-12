#!/usr/bin/python3

import os
import carla


def main():
    host = os.environ.get("WINDOWS_HOST")
    client = carla.Client(host, 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    actors = world.get_actors()
    print(actors)
    for actor in actors:
        print(actor)
        if "sensor." in actor.type_id or "vehicle." in actor.type_id:
            print(f'Destroying: {actor}')
            actor.destroy()


if __name__ == "__main__":
    main()
