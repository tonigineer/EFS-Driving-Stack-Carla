import numpy as np
from typing import Dict

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from carla_efs_api import CarlaAPI
from carla_efs_api.dashboard_standalone import DashboardStandalone, HUD
from carla_efs_api.ros_logging import loginfo

from carla_efs_messages.msg import (
    VehicleControl, ControllerStatus, ControllerHorizon
)


class Dashboard(Node):

    REFRESH_RATE_HZ = 72  # max. 144Hz set by display manager

    # NOTE Image sizes must be a multiple of 64, due to a bug.
    # https://github.com/carla-simulator/carla/issues/6085

    sensor_data = {}

    def __init__(self):
        super().__init__('carla_efs_dashboard')

        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.world = CarlaAPI.get_world()
        self.actor = CarlaAPI.get_actor(
            world=self.world, pattern=[self.role_name])

        self.set_up_communication()

        self.create_dashboard()

        self.timer_render = self.create_timer(
            1.0/self.REFRESH_RATE_HZ, self.dashboard.render_manually
        )

    def set_up_communication(self):
        # // SUBSCRIBER
        self.sub_odom = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self.callback_odometry, 10
        )

        self.sub_imu = self.create_subscription(
            Imu, f'/carla/{self.role_name}/imu',
            self.callback_imu, 10
        )

        self.sub_gnss = self.create_subscription(
            NavSatFix, f'/carla/{self.role_name}/gnss',
            self.callback_gnss, 10
        )

        self.sub_veh_ctrl = self.create_subscription(
            VehicleControl, f'/carla/{self.role_name}/vehicle_control',
            self.callback_veh_ctrl, 10
        )

        self.sub_ctrl_status = self.create_subscription(
            ControllerStatus, f'/carla/{self.role_name}/controller/status',
            self.callback_ctrl_status, 10
        )

    def callback_odometry(self, msg):
        self.sensor_data['odometry'] = msg

    def callback_imu(self, msg):
        self.sensor_data['imu'] = msg

    def callback_gnss(self, msg):
        self.sensor_data['gnss'] = msg

    def callback_ctrl_status(self, msg):
        self.sensor_data['ctrl_status'] = msg

    def callback_veh_ctrl(self, msg):
        self.sensor_data['veh_ctrl'] = msg

    def sensor_data_callback(self) -> Dict:
        return self.sensor_data

    def create_dashboard(self):
        loginfo('Dashboard started')

        width = 1280
        height = 720

        hud = HUD(
            actor=self.actor, world=self.world,
            width=width, height=height,
            sensor_callback=self.sensor_data_callback
        )

        self.dashboard = DashboardStandalone(
            actor=self.actor, world=self.world,
            width=width, height=height,
            hud_callback=hud.render,
        )

        self.dashboard.render_manually()

    def destroy(self):
        # NOTE: display_manager takes care of destroying sensory
        pass


def main(args=None):
    rclpy.init(args=args)

    dashboard = None
    try:
        dashboard = Dashboard()
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if dashboard:
            dashboard.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
