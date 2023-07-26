import carla
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

from carla_efs_messages.msg import ControllerHorizon
from carla_efs_api import CarlaAPI
from carla_efs_api.ros_logging import loginfo


class Painter(Node):

    SHOW_ROUTE = True
    SHOW_REFERENCE = True
    SHOW_HORIZON = True

    REFRESH_RATE_ROUTE_HZ = 0.2
    REFRESH_RATE_REFERENCE_HZ = 1
    REFRESH_RATE_HORIZON_HZ = 10

    def __init__(self):
        super().__init__('carla_efs_painter')

        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.world = CarlaAPI.get_world()

        self.configure_subscriber()

    def configure_subscriber(self) -> None:
        self.sub_route = self.create_subscription(
            Path, f'/carla/{self.role_name}/planner/route',
            self.callback_route, 10
        )

        self.sub_reference = self.create_subscription(
            Path, f'/carla/{self.role_name}/planner/reference',
            self.callback_reference, 10
        )

        self.sub_horizon = self.create_subscription(
            ControllerHorizon, f'/carla/{self.role_name}/controller/horizon',
            self.callback_horizon, 10
        )

    def callback_route(self, msg) -> None:
        if not self.SHOW_ROUTE:
            return

        nodes = np.array(
            [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
        )

        CarlaAPI.draw_debug_line(
            points=nodes,
            world=self.world,
            life_time=1.0/(self.REFRESH_RATE_ROUTE_HZ*0.75),
            location_z=0.01,
            thickness=0.20,
            color=carla.Color(1, 0, 0, 100)
        )

        loginfo('Route painted')

    def callback_reference(self, msg) -> None:
        if not self.SHOW_REFERENCE:
            return

        nodes = np.array(
            [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
        ).T

        CarlaAPI.draw_debug_line(
            points=nodes,
            world=self.world,
            life_time=1.0/(self.REFRESH_RATE_REFERENCE_HZ*0.75),
            location_z=0.05,
            thickness=0.25,
            color=carla.Color(0, 0, 1, 100)
        )
        loginfo('Reference painted')

    def callback_horizon(self, msg) -> None:
        if not self.SHOW_HORIZON:
            return

        nodes = np.array(
            [[x, -y] for (x, y) in zip(msg.x_position, msg.y_position)]
        ).T

        CarlaAPI.draw_debug_line(
            points=nodes,
            world=self.world,
            life_time=1.0/(self.REFRESH_RATE_HORIZON_HZ*0.75),
            location_z=0.15,
            thickness=0.25,
            color=carla.Color(1, 1, 1, 100)
        )

        loginfo('Horizon painted')

    def destroy(self) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)

    painter = None
    try:
        painter = Painter()
        rclpy.spin(painter)
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if painter:
            painter.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
