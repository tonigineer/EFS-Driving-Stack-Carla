#!/usr/bin/python3

"""A collection of Carla functionality as classmethods.

NOTE: The goal of this class it to provide a wide variey of
functionality to ROS Nodes as well as scripts. The design
decisions aim to achieve the following:
    - provide functionality without instantiating, e.g.:
      for one-liner commands or short scripts
    - get the number of connections as low as possible,
      when used within a node repeatedly (provide already
      connected client to function)
"""

import os
import re
import carla
import numpy as np
from typing import Dict, List
from random import choice
from time import sleep

from carla_msgs.msg import CarlaStatus

from ros_compatibility.exceptions import ROSException
from ros_compatibility import loginfo, logerr, logwarn
from rclpy.node import Node


class CarlaAPI():

    host = os.environ.get("WINDOWS_HOST")
    port = 2000
    timeout = 15

    @classmethod
    def get_client(cls) -> carla.Client:
        try:
            carla_client = carla.Client(host=cls.host, port=cls.port)
            carla_client.set_timeout(cls.timeout)
        except RuntimeError as e:
            logerr(f'Error while connecting to Carla: {e}')
            raise e

        loginfo("CARLA_API: Connected to Carla.")

        return carla_client

    @classmethod
    def get_world(cls, client=None) -> carla.World:
        if not client:
            client = cls.get_client()

        world = client.get_world()
        world.wait_for_tick()  # needed, so that actors are present
        loginfo("CARLA_API: Got world")
        return world

    @classmethod
    def get_actors(cls,
                   world=None,
                   type_id=None,
                   role_name=None) -> List[carla.Actor]:
        if not world:
            world = cls.get_world()

        actors = []
        for actor in world.get_actors():
            if actor.type_id == type_id or \
               actor.attributes.get("role_name") == role_name:
                loginfo(f'CARLA_API: Matched {actor}')
                actors.append(actor)

        if not actors:
            raise LookupError(f'No actor found with `type_id`: {type_id}' +
                              f'or `role_name`: {role_name}')

        return actors

    @classmethod
    def draw_debug_line(cls,
                        points: np.ndarray,
                        world: carla.World = None,
                        thickness: float = 0.25,
                        life_time: float = 0.75,
                        color: carla.Color = carla.Color(1, 0, 0, 0),
                        location_z: float = 0.15
                        ):
        if points.shape[1] != 2:
            if points.shape[0] != 2:
                raise TypeError(
                    f'Dimension mismatch - neither column or row size == 2'
                )
            points = points.T

        if not world:
            world = cls.get_world()

        for i in range(1, points.shape[0]):
            world.debug.draw_line(
                carla.Location(points[i-1, 0], points[i-1, 1], z=location_z),
                carla.Location(points[i, 0], points[i, 1], z=location_z),
                thickness=thickness,
                life_time=life_time,
                color=color
            )

    @staticmethod
    def convert_waypoints_to_array(waypoints: List[carla.Waypoint]
                                   ) -> np.ndarray:
        """Convert waypoints to array with x-y coordinates."""
        return np.array(
            [
                [wp[0].transform.location.x, wp[0].transform.location.y]
                for wp in waypoints
            ]
        )

    @classmethod
    def set_delta_seconds(cls, world: carla.World = None,
                          fps: int = 60) -> None:
        """Set server sample time."""
        if not world:
            world = cls.get_world()

        settings = world.get_settings()

        settings.fixed_delta_seconds = 1/fps
        settings.synchronous_mode = False

        world.apply_settings(settings)

        print(
            f'CARLA_API: fixed_delta_seconds = {settings.fixed_delta_seconds}'
        )

    @classmethod
    def get_actors(cls, pattern: List[str],
                   world: carla.World = None) -> List[carla.Actor]:
        """Return actors with matching pattern for `type_id` or `role_name`."""
        if not world:
            world = cls.get_world()

        if type(pattern) is not list:
            logwarn(f'keyword argmeunte `pattern` put in a list')
            pattern = [pattern]

        actors = []
        for actor in world.get_actors():
            matched = False
            if any(p in actor.type_id for p in pattern):
                matched = True
            if actor.attributes.get('role_name'):
                if any(p in actor.attributes.get('role_name') for p in pattern):
                    matched = True
            if matched:
                actors.append(actor)

        return actors

    @classmethod
    def remove_actors(cls,
                      pattern: List[str],
                      world: carla.World = None) -> None:
        """Remove actors with matching pattern."""
        actors = cls.get_actors(pattern=pattern, world=world)
        for actor in actors:
            loginfo(f'Removed actor: {actor}')
            actor.destroy()

    @classmethod
    def move_to_actor(cls, pattern, world: carla.World = None) -> None:
        """Move spectator of Carla main window to actor."""
        if not world:
            world = cls.get_world()

        actors = cls.get_actors(pattern, world)

        if not actors:
            logerr(f'No actor found for `pattern`: {pattern}')
            return None

        if len(actors) > 1:
            logwarn(f'Multiple actors found for `pattern`: {pattern}')

        loc = actors[0].get_transform().location
        rot = actors[0].get_transform().rotation

        offset = -6
        dx = offset * np.cos(np.deg2rad(rot.yaw))
        dy = offset * np.sin(np.deg2rad(rot.yaw))

        transform = carla.Transform(
            carla.Location(loc.x + dx, loc.y + dy, loc.z + 3),
            carla.Rotation(rot.pitch - 15, rot.yaw, rot.roll)
        )

        world.get_spectator().set_transform(transform)

    @classmethod
    def spawn_vehicle(cls, blueprint: str = 'etron',
                      role_name: str = 'ego_vehicle',
                      spawn_point: carla.Transform = None,
                      world: carla.World = None):
        if not world:
            world = cls.get_world()

        if not spawn_point:
            spawn_point = choice(world.get_map().get_spawn_points())

        bp_lib = world.get_blueprint_library()
        vehicle_bp = bp_lib.filter(blueprint)[0]
        vehicle_bp.set_attribute('role_name', role_name)

        vehilce = world.try_spawn_actor(vehicle_bp, spawn_point)

    @classmethod
    def print_all_actors(cls,
                         world: carla.World = None):
        if not world:
            world = cls.get_world()

        for actor in world.get_actors():
            print(actor)

    @classmethod
    def respawn_actor(cls,
                      world: carla.World = None,
                      role_name: str = 'ego_vehicle'):
        if not world:
            world = cls.get_world()

        actors = cls.get_actors(world=world, pattern=[role_name])
        actor = actors[0]

        # TODO: Quite inelegant to use a file, but otherwise a node
        # needs to be set up to get the information from ROS topic's.
        with open(f'./log/PLANNER_SPAWN_POINT_{role_name}') as f:
            line = f.readlines()[0]
        numbers = re.findall(r"[-+]?(?:\d*\.*\d+)", line)

        spawn_point = carla.Transform(
            carla.Location(*map(float, numbers[0:3])),
            carla.Rotation(*map(float, numbers[3:6]))
        )
        actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
        actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
        actor.set_transform(spawn_point)

    @classmethod
    def pin_spectator(cls,
                      world: carla.World = None,
                      pattern: List[str] = ['ego_vehicle']):
        if not world:
            world = cls.get_world()

        actor = cls.get_actors(world=world, pattern=pattern)[0]

        try:
            while True:
                loc = actor.get_transform().location
                rot = actor.get_transform().rotation

                offset = -6
                dx = offset * np.cos(np.deg2rad(rot.yaw))
                dy = offset * np.sin(np.deg2rad(rot.yaw))

                transform = carla.Transform(
                    carla.Location(loc.x + dx, loc.y + dy, loc.z + 3),
                    carla.Rotation(rot.pitch - 15, rot.yaw, rot.roll)
                )

                world.get_spectator().set_transform(transform)

                sleep(1/120)
        except KeyboardInterrupt:
            print("User requested shut down.")


if __name__ == "__main__":
    # CarlaAPI.remove_actors(pattern=['ego_vehicle'])
    # CarlaAPI.spawn_vehicle(blueprint='vehicle.audi.etron',
    #                        role_name='ego_vehicle')
    # CarlaAPI.move_to_actor(pattern=['ego_vehicle'])

    # CarlaAPI.respawn_actor()

    CarlaAPI.pin_spectator()
