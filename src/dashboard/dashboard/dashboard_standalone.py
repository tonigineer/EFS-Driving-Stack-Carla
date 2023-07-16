"""Dashboard for Carla to visualize Simulation."""

import os
import sys
import carla
import numpy as np
import pygame as pg

from planner import CarlaAPI

from time import perf_counter, sleep
from dataclasses import dataclass
from typing import Tuple, List
from typing import Callable


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

    https://www.pygame.org/docs/ref/color_list.html
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
        fps_text = self.font.render(fps, 1, pg.Color("springgreen4"))
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

    def render(self, actor_infos: List[str] = None):
        OFFSET_X = 10
        OFFSET_Y_NEWLINE = 15

        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        info_surface = pg.Surface((150, 60))
        info_surface.set_alpha(100)
        self.display.blit(info_surface, (0, 0))

        self.display.blit(self.update_fps(), (OFFSET_X, 0))

        for i, info in enumerate(actor_infos):
            self.display.blit(
                self.font.render(
                    info, 1, pg.Color("slateblue")
                ), (OFFSET_X, (i+1) * OFFSET_Y_NEWLINE)
            )

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


class Dashboard:

    actor: carla.Actor
    actor_callback: Callable

    # NOTE Image sizes must be a multiple of 64, due to a bug.
    # https://github.com/carla-simulator/carla/issues/6085
    height: int

    world: carla.World

    def __init__(self,
                 actor: carla.Actor,
                 actor_callback: Callable = None,
                 height: int = 256,
                 world: carla.World = None):
        self.actor = actor
        self.actor_callback = actor_callback

        self.height = height

        self.world = world if world else CarlaAPI.get_world()

        # TODO: Implemented interface to change setup.
        self.sensory = [
            SensorSetup(actor, "RGBCamera", (0, 0, 10), (0, -90, 0)),
            SensorSetup(actor, "RGBCamera", (-5, 0, 3.5), (0, -10, 0)),
            # SensorSetup(actor, "RGBCamera", (-1, 2, 2.4), (0, -8, -30)),
        ]

        self.create_display_manager()

    def create_display_manager(self):
        """Create display manager for actor with sensory setup."""
        self.display_manager = DisplayManager(
            grid_size=[1, len(self.sensory)],
            window_size=[self.height*len(self.sensory), self.height]
        )

        for idx, sensor in enumerate(self.sensory):
            SensorManager(
                self.world, self.display_manager, sensor.sensor_type,
                carla.Transform(
                    carla.Location(
                        x=sensor.x, y=sensor.y, z=sensor.z
                    ),
                    carla.Rotation(
                        roll=sensor.roll, pitch=sensor.pitch, yaw=sensor.yaw
                    )
                ), sensor.actor, {}, display_pos=[0, idx]
            )

    def start(self):
        """Start Dashboard loop to render camera and info data."""
        try:
            while True:
                self.display_manager.render(
                    actor_infos=self.actor_callback()
                )
                if self.display_manager.check_events():
                    break
        finally:
            self.display_manager.destroy()

    def render_manually(self):
        self.display_manager.render(
            actor_infos=self.actor_callback()
        )


def main():
    """Entry point for example."""
    actor = CarlaAPI.get_actors(pattern=['ego_vehicle'])[0]

    def _callback_info_text():
        """Exemplary callback for actor information."""
        return ['line 1', 'line 2']

    dashboard = Dashboard(
        actor=actor,
        actor_callback=_callback_info_text
    )

    dashboard.start()


if __name__ == "__main__":
    main()
