import carla
import numpy as np
import pygame as pg
from typing import List

from planner import CarlaAPI
from dashboard import Dashboard
from messages.msg import StatusMPC

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from ros_compatibility.exceptions import ROSException


class DashboardNode(CompatibleNode):

    dashboard = None

    odometry = None
    ackermann = None
    status_mpc = None

    world = CarlaAPI.get_world()

    def __init__(self):
        super().__init__('dashboardnode')
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.set_up_communication()
        self.actor = CarlaAPI.get_actors(
            world=self.world, pattern=[self.role_name]
        )[0]
        self.create_dashboard()

        self.timer_render = self.create_timer(
            0.01, self.dashboard.render_manually
        )

    def set_up_communication(self):
        # // SUBSCRIBER
        self.sub_odom = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self._callback_odometry, 10
        )

        self.sub_ackermann = self.create_subscription(
            AckermannDrive, f'/carla/{self.role_name}/ackermann_cmd',
            self._callback_ackermann, 10
        )

        self.sub_status_mpc = self.create_subscription(
            StatusMPC, f'/carla/{self.role_name}/status_mpc',
            self._callback_status_mpc, 10
        )

    def _callback_odometry(self, msg):
        self.odometry = msg

    def _callback_ackermann(self, msg):
        self.ackermann = msg

    def _callback_status_mpc(self, msg):
        self.status_mpc = msg

    def _actor_callback(self) -> List[str]:
        infos = []

        if not self.odometry or not self.ackermann or not self.status_mpc:
            return infos

        infos.append(
            f'vx_ego: {np.abs(self.odometry.twist.twist.linear.x):0.2f}   m/s')
        infos.append(
            f'vx_set: {self.ackermann.speed:0.2f}   m/s')
        infos.append(f'')
        infos.append(
            f'delta_y: {self.status_mpc.lateral_deviation:0.2f}  m')
        infos.append(
            f't_exec : {self.status_mpc.execution_time:0.3f} s')

        return infos

    def create_dashboard(self):
        self.loginfo('Dashboard started')

        self.dashboard = Dashboard(
            actor=self.actor,
            actor_callback=self._actor_callback,
            height=384
        )

        self.dashboard.render_manually()


def main(args=None):
    """Start basic ROS2 main function `dashboard` node."""
    roscomp.init('dashboardnode', args)

    dash = None
    try:
        dash = DashboardNode()
        dash.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if dash:
            if dash.dashboard:
                dash.dashboard.display_manager.destroy()
            dash.destroy()
        roscomp.shutdown()


if __name__ == '__main__':
    main()
