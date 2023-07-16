import carla
import numpy as np
from typing import List

from planner import CarlaAPI
from dashboard import Dashboard

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from ros_compatibility.exceptions import ROSException


class DashboardNode(CompatibleNode):

    dashboard = None

    odometry = None
    ackermann = None

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

    def _callback_odometry(self, msg):
        # self.loginfo('Odometry received')
        self.odometry = msg

    def _callback_ackermann(self, msg):
        # NOTE: this message is published by the controller
        self.ackermann = msg

    def _actor_callback(self) -> List[str]:
        infos = []

        if not self.odometry or not self.ackermann:
            return infos

        infos.append(f'vx: {self.odometry.twist.twist.linear.x:0.2f} m/s')
        infos.append(f'vx: {self.ackermann.speed:0.2f} m/s')

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
