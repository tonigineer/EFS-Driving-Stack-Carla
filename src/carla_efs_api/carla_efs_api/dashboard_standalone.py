import os
import carla
import numpy as np
import pygame as pg
import datetime
import math

from carla_efs_api import CarlaAPI
from carla_efs_api.ros_logging import logwarn

from time import perf_counter
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
    MAX_FPS = 144

    sensor_list = []

    def __init__(self, grid_size: Tuple[int, int],
                 window_size: Tuple[int, int]):
        pg.init()
        pg.font.init()

        self.clock = pg.time.Clock()
        self.font = pg.font.SysFont("FreeMono, Monospace", 18)

        self.display = pg.display.set_mode(
            window_size, pg.HWSURFACE | pg.DOUBLEBUF
        )

        self.grid_size = grid_size
        self.window_size = window_size

    def update_fps(self):
        fps = f'FPS: {int(self.clock.get_fps())}'
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

    def render(self, hud_callback: Callable = None):
        # OFFSET_X = 10
        # OFFSET_Y_NEWLINE = 15

        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        if hud_callback:
            self.display = hud_callback(self.display, self.clock)

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


class DashboardStandalone:

    actor: carla.Actor
    sensor_callback: Callable

    # NOTE Image sizes must be a multiple of 64, due to a bug.
    # https://github.com/carla-simulator/carla/issues/6085

    world: carla.World

    def __init__(self,
                 actor: carla.Actor,
                 world: carla.World,
                 width: int = 1280,
                 height: int = 720,
                 hud_callback: Callable = None):
        self.actor = actor
        self.world = world
        self.width = width
        self.height = height
        self.hud_callback = hud_callback

        # TODO: Implemented interface to change setup.
        self.sensory = [
            # SensorSetup(actor, "RGBCamera", (0, 0, 10), (0, -90, 0)),
            SensorSetup(actor, "RGBCamera", (-5, 0, 3.5), (0, -10, 0)),
            # SensorSetup(actor, "RGBCamera", (-1, 2, 2.4), (0, -8, -30)),
        ]

        self.create_display_manager()

    def create_display_manager(self):
        """Create display manager for actor with sensory setup."""
        self.display_manager = DisplayManager(
            grid_size=[1, len(self.sensory)],
            window_size=[self.width, self.height]
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
                self.display_manager.render(self.hud_callback)
                if self.display_manager.check_events():
                    break
        finally:
            self.display_manager.destroy()

    def render_manually(self):
        # NOTE: Checking events is needed, otherwise Pygame
        # shows `not responding error` while still working.
        # Only got this error on my Ubuntu-22.04 so far.
        self.display_manager.render(self.hud_callback)
        if self.display_manager.check_events():
            self.display_manager.destroy()
            raise KeyboardInterrupt('Check event found to shutdown dashboard.')


class HUD:

    server_fps = 0
    sim_time = 0

    server_clock = None

    BAR_H_OFFSET = 95
    BAR_WIDTH = 120

    ROW_Y_OFFSET = 18

    _info_surface = None

    def __init__(self, actor, world, width, height,
                 sensor_callback: Callable = None):
        pg.font.init()

        self.actor = actor
        self.world = world
        self.world.on_tick(self.on_world_tick)

        self.dim = (width, height)
        self.font = self.define_font()

        self.sensor_callback = sensor_callback

        self.server_clock = pg.time.Clock()

    @staticmethod
    def define_font():
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pg.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pg.font.match_font(mono)
        return pg.font.Font(mono, 12 if os.name == 'nt' else 14)

    def on_world_tick(self, timestamp):
        if not self.server_clock:
            return
        self.server_clock.tick()
        self.server_fps = self.server_clock.get_fps()
        self.sim_time = timestamp.elapsed_seconds

    def gather_information(self, clock) -> List:
        # Get from ROS topic's (actually also from Carla ;))
        velocity = 'missing topic'
        compass = 'missing topic'
        gyroscope = 'missing topic'
        gnss = 'missing topic'
        imu = 'missing topic'

        lat_dev = 'missing topic'
        v_diff = 'missing topic'
        exec_time = 'missing topic'
        mpc_ax = 'missing topic'
        mpc_vx = 'missing topic'
        mpc_deltav = 'missing topic'

        if self.sensor_callback:
            data = self.sensor_callback()
            if 'odometry' in data.keys():
                velocity = \
                    f"{data['odometry'].twist.twist.linear.x*3.6:0.0f} km/h"

            if 'gnss' in data.keys():
                gnss = f"({data['gnss'].latitude:0.5f}, " + \
                       f"{data['gnss'].longitude:0.5f})"

            if 'imu' in data.keys():
                imu = f"({data['imu'].linear_acceleration.x:+2.1f}," + \
                      f" {data['imu'].linear_acceleration.y:+2.1f}," + \
                      f" {data['imu'].linear_acceleration.z:+2.1f})"

                gyroscope = \
                    f"({data['imu'].angular_velocity.x:+2.1f}," + \
                    f" {data['imu'].angular_velocity.y:+2.1f}," + \
                    f" {data['imu'].angular_velocity.z:+2.1f})"

            if 'ctrl_status' in data.keys():
                exec_time = f"{data['ctrl_status'].execution_time*1000:0.0f} ms"
                lat_dev = f"{data['ctrl_status'].lateral_deviation:0.2f} m"
                v_diff = \
                    f"{data['ctrl_status'].velocity_difference*3.6:0.1f} km/h"

            if 'veh_ctrl' in data.keys():
                mpc_ax = f"{data['veh_ctrl'].desired_acceleration:0.1f} m/ss"
                mpc_vx = f"{data['veh_ctrl'].desired_velocity*3.6:0.0f} km/h"
                mpc_deltav = \
                    f"{data['veh_ctrl'].desired_steering_angle:0.3f} rad"

        # Read from Carla directly
        t = self.actor.get_transform()
        location = f"({t.location.x:0.1f}, {t.location.y:0.1f})"
        c = self.actor.get_control()
        altitude = f"{t.location.z:0.1f}"

        gear = {-1: "R", 0: "N"}.get(c.gear, c.gear)

        # Build list of strings from data
        information = [
            f'Server:   {self.server_fps:12.0f} FPS',
            f'Client:   {clock.get_fps():12.0f} FPS',
            '',
            f'Vehicle:  {" ".join(self.actor.type_id.replace("_", ".").title().split(".")[1:]): >16}',
            f'Map:      {self.world.get_map().name.split("/")[-1]: >16}',
            '',
            f'Simulation time:   {datetime.timedelta(seconds=int(self.sim_time))}',
            '',
            f'Speed:    {velocity: >16}',
            f'IMU:    {imu: >16}',
            f'Gyro:   {gyroscope: >16}',
            f'Compasss: {compass: >16}',
            f'Location: {location: >16}',
            f'GNSS:   {gnss: >16}',
            f'Altitude: {altitude: >14} m',
            '',
            ('Throttle: ', c.throttle, 0.0, 1.0),
            ('Steer:    ', c.steer, -1.0, 1.0),
            ('Brake:    ', c.brake, 0.0, 1.0),
            f'Gear:      {gear}',
            '',
            (153, 0, 0),  # line
            '',
            f'delta_y: {lat_dev: >17}',
            f'vx_diff: {v_diff: >17}',
            '',
            f'MPC time: {exec_time: >16}',
            f'MPC ax: {mpc_ax: >18}',
            f'MPC vx: {mpc_vx: >18}',
            f'MPC delta_v: {mpc_deltav: >13}'
        ]

        return information

    def render(self, display, clock):
        # Background
        if not self._info_surface:
            self._info_surface = pg.Surface((220, self.dim[1]))
            self._info_surface.set_alpha(150)
        display.blit(self._info_surface, (0, 0))

        information = self.gather_information(clock)

        y_offset = 4
        for item in information:
            if y_offset + self.ROW_Y_OFFSET > self.dim[1]:
                break

            if isinstance(item, tuple):
                # Item is color code for a line
                if all(isinstance(v, int) for v in item):
                    pg.draw.lines(
                        display, item,
                        False, [(10, y_offset), (210, y_offset)], 2)
                    continue

                # Draw bars for pedals
                rect_border = pg.Rect((self.BAR_H_OFFSET, y_offset + 8), (self.BAR_WIDTH, 6))
                pg.draw.rect(display, (255, 255, 255), rect_border, 1)
                f = (item[1] - item[2]) / (item[3] - item[2])
                if item[2] < 0.0:
                    rect = pg.Rect((self.BAR_H_OFFSET + f * (self.BAR_WIDTH - 6), y_offset + 8), (6, 6))
                else:
                    rect = pg.Rect((self.BAR_H_OFFSET, y_offset + 8), (f * self.BAR_WIDTH, 6))
                pg.draw.rect(display, (255, 255, 255), rect)

                item = item[0]

            surface = self.font.render(item, True, (255, 255, 255))
            display.blit(surface, (8, y_offset))

            y_offset += self.ROW_Y_OFFSET
        return display


def main():
    """Entry point for example."""
    pg.init()
    pg.font.init()
    world = CarlaAPI.get_world()
    actor = CarlaAPI.get_actor(world=world, pattern=['ego_vehicle'])

    width = 1280
    height = 720

    hud = HUD(
        actor=actor, world=world,
        width=width, height=height,
        sensor_callback=None
    )

    dashboard = DashboardStandalone(
        actor=actor, world=world,
        width=width, height=height,
        hud_callback=hud.render
    )

    dashboard.start()


if __name__ == "__main__":
    main()
