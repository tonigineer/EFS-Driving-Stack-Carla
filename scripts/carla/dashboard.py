"""Dashboard for Carla to visualize Simulation."""

import os
import sys
import carla
import numpy as np
import pygame as pg

from time import perf_counter, sleep
from dataclasses import dataclass
from typing import Tuple


@dataclass
class SensorSetup:

    actor: carla.Actor
    sensor_type: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def __init__(self, actor: carla.Actor,
                 sensor_type: str, xyz: Tuple[float], rpy: Tuple[float]):
        self.actor = actor
        self.sensor_type = sensor_type
        self.x, self.y, self.z = xyz
        self.roll, self.pitch, self.yaw = rpy


class DisplayManager:
    """Pygame framework to display sensor data.

    Taken from Carla example for `visualization`.
    Small adjustments made though.
    """

    EXIT_KEYS = [pg.K_ESCAPE, pg.K_q]
    MAX_FPS = 60

    sensor_list = []

    def __init__(self, grid_size: Tuple[int, int],
                 window_size: Tuple[int, int]):
        pg.init()
        pg.font.init()

        self.clock = pg.time.Clock()
        self.font = pg.font.SysFont("Arial", 18)

        self.display = pg.display.set_mode(
            window_size, pg.HWSURFACE | pg.DOUBLEBUF
        )

        self.grid_size = grid_size
        self.window_size = window_size

    def update_fps(self):
        fps = str(int(self.clock.get_fps()))
        fps_text = self.font.render(fps, 1, pg.Color("coral"))
        return fps_text

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [
            int(self.window_size[0]/self.grid_size[1]),
            int(self.window_size[1]/self.grid_size[0])
        ]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return
        for s in self.sensor_list:
            s.render()

        self.display.blit(self.update_fps(), (10, 0))

        pg.display.flip()
        self.clock.tick(self.MAX_FPS)

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display

    def check_events(self) -> bool:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return True
            elif event.type == pg.KEYDOWN:
                pg.K_ESCAPE
                if event.key in self.EXIT_KEYS:
                    return True
        return False


class SensorManager:
    """Handler for sensors placed as actors in Carla.

    Taken from Carla example for `visualization`.
    Small adjustments made though.
    """

    def __init__(self, world, display_man, sensor_type, transform,
                 attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(
            sensor_type, transform, attached, sensor_options
        )
        self.sensor_options = sensor_options

        self.frame_rate = 0.0
        self.frame = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(
                camera_bp, transform, attach_to=attached
            )
            camera.listen(self.save_rgb_image)

            return camera
        else:
            return None

    def save_rgb_image(self, image):
        t_start = perf_counter()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pg.surfarray.make_surface(array.swapaxes(0, 1))

        self.frame += 1
        self.frame_rate = 1 / (perf_counter() - t_start)

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


class CarlaApi:

    HOST = os.environ.get("WINDOWS_HOST")
    PORT = 2000
    TIMEOUT = 10.0

    def __init__(self):
        self.client = self.connect()

    @classmethod
    def connect(cls) -> carla.Client:
        client = carla.Client(cls.HOST, cls.PORT)
        client.set_timeout(cls.TIMEOUT)
        return client

    def get_actor(self, role_name) -> carla.Actor:
        world = self.client.get_world()
        world.wait_for_tick()  # needed, so that the actors are present
        actors = world.get_actors()

        if len(actors) == 0:
            raise ValueError('Actors were not populated in time or at all.')

        for actor in actors:
            # print(actor.attribut)
            if actor.attributes.get('role_name') == role_name:
                return actor
        raise ValueError(f'Could not get actor with role-name=`{role_name}`')


class Monitor:

    HOST = os.environ.get("windows_host")
    PORT = 2000
    TIMEOUT = 2.0

    # TYPE_ID_RABIT = "vehicle.audi.a2"
    # TYPE_ID_FOX = "vehicle.audi.etron"

    def __init__(self):
        self.world = CarlaApi.connect().get_world()

    def initiate_monitor(self, sensory: SensorSetup,
                         disp_size):

        display_manager = DisplayManager(
            grid_size=[1, len(sensory)],
            window_size=disp_size
        )

        for idx, sensor in enumerate(sensory):
            SensorManager(
                self.world, display_manager, sensor.sensor_type,
                carla.Transform(
                    carla.Location(
                        x=sensor.x, y=sensor.y, z=sensor.z
                    ),
                    carla.Rotation(
                        roll=sensor.roll, pitch=sensor.pitch, yaw=sensor.yaw
                    )
                ), sensor.actor, {}, display_pos=[0, idx]
            )

        try:
            while True:
                display_manager.render()
                if display_manager.check_events():
                    break
        finally:
            display_manager.destroy()

    def show(self):
        ca = CarlaApi()

        # NOTE Image sizes must be a multiple of 64, due to a bug.
        # https://github.com/carla-simulator/carla/issues/6085
        HEIGHT = 256

        # Settings for Monitor
        if len(sys.argv) > 1:
            raise NotImplementedError('implement now!')
        else:
            vehicle_id = ca.get_actor('ego_vehicle')

        sensory = [
            SensorSetup(vehicle_id, "RGBCamera", (0, 0, 10), (0, -90, 0)),
            SensorSetup(vehicle_id, "RGBCamera", (-4, 0, 2.4), (0, -8, 0)),
            SensorSetup(vehicle_id, "RGBCamera", (-1, 2, 2.4), (0, -8, -30)),
        ]

        self.initiate_monitor(sensory, disp_size=[HEIGHT*len(sensory), HEIGHT])


def main():
    monitor = Monitor()
    monitor.show()


if __name__ == "__main__":
    main()
