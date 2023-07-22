import carla
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

from carla_efs_api import CarlaAPI, loginfo


class Painter(Node):

    REFRESH_RATE_ROUTE_HZ = 0.2
    REFRESH_RATE_REFERENCE_HZ = 1

    def __init__(self):
        super().__init__('carla_efs_painter')

        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.world = CarlaAPI.get_world()

        self.configure_subscriber()

    def callback_route(self, msg):
        nodes = np.array(
            [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
        )

        CarlaAPI.draw_debug_line(
            points=nodes,
            world=self.world,
            life_time=1.0/self.REFRESH_RATE_ROUTE_HZ,
            location_z=0.01,
            thickness=0.20,
            color=carla.Color(1, 0, 0, 100)
        )

        loginfo('Route painted')

    def callback_reference(self, msg):
        nodes = np.array(
            [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
        ).T

        CarlaAPI.draw_debug_line(
            points=nodes,
            world=self.world,
            life_time=1.0/self.REFRESH_RATE_REFERENCE_HZ,
            location_z=0.05,
            thickness=0.25,
            color=carla.Color(1, 1, 1, 100)
        )
        loginfo('Reference painted')

    def configure_subscriber(self) -> None:
        self.sub_route = self.create_subscription(
            Path, f'/carla/{self.role_name}/planner/route',
            self.callback_route, 10
        )

        self.sub_reference = self.create_subscription(
            Path, f'/carla/{self.role_name}/planner/reference',
            self.callback_reference, 10
        )


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
