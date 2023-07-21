import rclpy
import numpy as np

from typing import List

from nav_msgs.msg import Odometry

from ros_compatibility import loginfo
from ros_compatibility.exceptions import ROSException

from carla_efs_api import CarlaAPI, DashboardStandalone


class Dashboard(rclpy.node.Node):

    REFRESH_RATE_HZ = 72  # max. 144Hz set by display manager

    # NOTE Image sizes must be a multiple of 64, due to a bug.
    # https://github.com/carla-simulator/carla/issues/6085
    CAMERA_WIDTH = 64 * 4

    odometry = None

    def __init__(self):
        super().__init__('carla_efs_dashboard')

        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.world = CarlaAPI.get_world()
        self.actor = CarlaAPI.get_actors(
            world=self.world, pattern=[self.role_name])[0]

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

        # self.sub_ackermann = self.create_subscription(
        #     AckermannDrive, f'/carla/{self.role_name}/ackermann_cmd',
        #     self._callback_ackermann, 10
        # )

        # self.sub_status_mpc = self.create_subscription(
        #     StatusMPC, f'/carla/{self.role_name}/status_mpc',
        #     self._callback_status_mpc, 10
        # )

    def callback_odometry(self, msg):
        # loginfo('fdasfdsa')
        self.odometry = msg

    # def callback_ackermann(self, msg):
    #     self.ackermann = msg

    # def callback_status_mpc(self, msg):
    #     self.status_mpc = msg

    def actor_callback(self) -> List[str]:
        infos = []

        if not self.odometry:
            return infos

        infos.append(
            f'vx_ego: {np.abs(self.odometry.twist.twist.linear.x):0.2f}   m/s')
        # infos.append(
        #     f'vx_set: {self.ackermann.speed:0.2f}   m/s')
        # infos.append(f'')
        # infos.append(
        #     f'delta_y: {self.status_mpc.lateral_deviation:0.2f}  m')
        # infos.append(
        #     f't_exec : {self.status_mpc.execution_time:0.3f} s')

        return infos

    def create_dashboard(self):
        loginfo('Dashboard started')

        self.dashboard = DashboardStandalone(
            actor=self.actor,
            actor_callback=self.actor_callback,
            height=self.CAMERA_WIDTH
        )

        self.dashboard.render_manually()

    def destroy(self):
        # NOTE: display_manager takes care of destroying sensory
        pass
        # self.dashboard.display_manager.destroy()

        # self.configure_subscriber()

    # def callback_route(self, msg):
    #     nodes = np.array(
    #         [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
    #     )

    #     CarlaAPI.draw_debug_line(
    #         points=nodes,
    #         world=self.world,
    #         life_time=1.0/self.REFRESH_RATE_ROUTE_HZ,
    #         location_z=0.01,
    #         thickness=0.20,
    #         color=carla.Color(1, 0, 0, 100)
    #     )

    #     loginfo('Route painted')

    # def callback_reference(self, msg):
    #     nodes = np.array(
    #         [[p.pose.position.x, -p.pose.position.y] for p in msg.poses]
    #     ).T

    #     CarlaAPI.draw_debug_line(
    #         points=nodes,
    #         world=self.world,
    #         life_time=1.0/self.REFRESH_RATE_REFERENCE_HZ,
    #         location_z=0.05,
    #         thickness=0.25,
    #         color=carla.Color(1, 1, 1, 100)
    #     )
    #     loginfo('route reference')

    # def configure_subscriber(self) -> None:
    #     self.sub_route = self.create_subscription(
    #         Path, f'/carla/{self.role_name}/planner/route',
    #         self.callback_route, 10
    #     )

    #     self.sub_reference = self.create_subscription(
    #         Path, f'/carla/{self.role_name}/planner/reference',
    #         self.callback_reference, 10
    #     )


def main(args=None):
    rclpy.init(args=args)

    dashboard = None
    try:
        dashboard = Dashboard()
        rclpy.spin(dashboard)
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if dashboard:
            dashboard.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
