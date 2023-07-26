import os
import numpy as np
from random import choice

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from carla_efs_api import CarlaAPI
import carla_efs_api.transformations as tf
from carla_efs_api.ros_logging import loginfo, logwarn
from carla_efs_planner import GlobalRoutePlanner


class Planner(Node):

    REFRESH_RATE_ROUTE_HZ = 0.2
    REFRESH_RATE_REFERENCE_HZ = 1

    REFERENCE_TIME_LENGTH_SEC = 5
    MIN_REFERENCE_LENGHT = 10

    MIN_NUM_WP_IN_ROUTE = 200
    MAX_NUM_WP_REMAIN_REPLANNING = 50

    odometry = None
    route = None
    nodes = None
    reference_indices = None

    _last_idx = 0

    def __init__(self):
        super().__init__('carla_efs_planner')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.configure_subscriber()
        self.configure_publisher()

        # TODO: Arguments needed :)
        self.world = CarlaAPI.get_world()
        self.actor = CarlaAPI.get_actor(
            world=self.world,
            pattern=[self.role_name]
        )

        # Initial route
        self.calculate_route()

    def configure_subscriber(self) -> None:
        self.sub_odom = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self.callback_odometry, 10
        )

    def configure_publisher(self) -> None:
        self.timer_route = self.create_timer(
            1.0/self.REFRESH_RATE_ROUTE_HZ,
            self.callback_route
        )
        self.pub_route = self.create_publisher(
            Path, f'/carla/{self.role_name}/planner/route', 10
        )

        self.timer_reference = self.create_timer(
            1.0/self.REFRESH_RATE_REFERENCE_HZ,
            self.callback_reference
        )
        self.pub_reference = self.create_publisher(
            Path, f'/carla/{self.role_name}/planner/reference', 10
        )

    def callback_route(self) -> None:
        if self.nodes is None:
            logwarn('Route not calculated yet.')
            return

        loginfo(
            f'Publish `route` topic with '
            f'{len(self.route)} waypoints'
        )

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for wp in self.route:
            pose = PoseStamped()
            pose.pose = tf.carla_transform_to_ros_pose(
                wp[0].transform
            )
            msg.poses.append(pose)

        self.pub_route.publish(msg)

    def callback_reference(self) -> None:
        self.calculate_reference()
        if self.reference_indices is None:
            logwarn('Reference not found yet')
            return

        loginfo(
            f'Publish `reference` topic with '
            f'{len(self.reference_indices)} nodes'
        )

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for idx in self.reference_indices:
            pose = PoseStamped()
            pose.pose = tf.carla_transform_to_ros_pose(
                self.route[idx][0].transform
            )
            msg.poses.append(pose)

        self.pub_reference.publish(msg)

    def callback_odometry(self, msg) -> None:
        self.odometry = msg

    def calculate_reference(self) -> None:
        """Return a part of the route based on current position."""
        self.reference_indices = None

        if self.odometry is None:
            logwarn(
                f'Odometry for {self.role_name} not published yet, '
                f'reference can not be provided.'
            )
            return None

        # Replanning when approaching and of route
        if self._last_idx + self.MAX_NUM_WP_REMAIN_REPLANNING >= self.nodes.shape[0]:
            self.calculate_route()

        ego_pos = np.array([
            [self.odometry.pose.pose.position.x,
             -self.odometry.pose.pose.position.y]
        ])

        dist_2 = np.sum((self.nodes - ego_pos)**2, axis=1)
        idx = np.argmin(dist_2)

        # Make sure to start reference behind current position
        idx -= min(idx, 3)
        self._last_idx = idx

        s = 0
        self.reference_indices = [idx]
        reference_length = max(
            self.REFERENCE_TIME_LENGTH_SEC*self.odometry.twist.twist.linear.x,
            self.MIN_REFERENCE_LENGHT
        )
        while s <= reference_length:
            idx += 1
            dx = self.nodes[idx, 0] - self.nodes[idx-1, 0]
            dy = self.nodes[idx, 1] - self.nodes[idx-1, 1]
            s += np.sqrt(dx*dx + dy*dy)
            self.reference_indices.append(idx)

    def calculate_route(self) -> None:
        """Create a route vom `actor` to `goal`.

        NOTE: Taken from
        https://github.com/carla-simulator/ros-bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py

        Don't know, why the function does not work from Repo, but in the end,
        main functionality can be salvaged :)
        """
        loginfo(f'Global route planner ...')

        while True:
            self.start = self.actor.get_location()
            self.goal = choice(
                self.world.get_map().get_spawn_points()
            ).location
            loginfo(f'{self.start} {self.goal}')

            grp = GlobalRoutePlanner(self.world.get_map(), sampling_resolution=1)
            self.route = grp.trace_route(
                self.start,
                self.goal
            )

            # NOTE: Selecting a fitting goal right at beginning would avoid
            # running the planner multiple times. Because the planner does not
            # take much time, this solution is okay.
            if len(self.route) > self.MIN_NUM_WP_IN_ROUTE:
                break

        loginfo(
            f'▹ Calculated route contains {len(self.route)} waypoints'
        )

        self.nodes = CarlaAPI.convert_waypoints_to_array(self.route)

        # Output spawnpoint of route for respawning, for random spawn points
        os.system(f'echo "{self.route[0][0].transform}" '
                  f'> ./log/PLANNER_SPAWN_POINT_{self.role_name}')

        self.callback_route()

    def destroy(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    planner = None
    try:
        planner = Planner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if planner:
            planner.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
