import os
import carla
import numpy as np

from planner import CarlaAPI

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from ros_compatibility.exceptions import ROSException

from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla_common.transforms as trans


class Planner(CompatibleNode):

    current_route = None
    actor = None
    world = None
    goal = None
    odometry = None

    nodes = None

    REFERENCE_MIN_LENGTH = 40

    def __init__(self):
        super().__init__('planner')
        # self._connect_to_carla()
        self.client = CarlaAPI.get_client()
        self.world = CarlaAPI.get_world(client=self.client)

        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.actor = CarlaAPI.get_actors(
            world=self.world, pattern=[self.role_name]
        )[0]
        self.goal = self.world.get_map().get_spawn_points()[0].location

        self.set_up_communication()

        self.calculate_route()

    def set_up_communication(self):
        # // PUBLISHER
        timer_period = 10.0  # seconds
        self.timer_route = self.create_timer(
            timer_period, self._callback_route
        )
        self.pub_path = self.create_publisher(
            Path, f'/carla/{self.role_name}/planner/route', 10
        )

        timer_period = 1.0  # seconds
        self.timer_reference = self.create_timer(
            timer_period, self._callback_reference
        )
        self.pub_reference = self.create_publisher(
            Path, f'/carla/{self.role_name}/planner/reference', 10
        )

        # // SUBSCRIBER
        self.sub_odom = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self._callback_odometry, 10
        )

    def _callback_odometry(self, msg):
        self.odometry = msg

    def _callback_route(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = roscomp.ros_timestamp(
            self.get_time(), from_sec=True
        )

        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
                msg.poses.append(pose)

        self.pub_path.publish(msg)
        CarlaAPI.draw_debug_line(
            points=self.nodes,
            world=self.world,
            life_time=10.5,
            location_z=0.01,
            thickness=0.2
        )
        self.loginfo(
            f'▹ Route visualized as line in Carla (10s life time)'
        )

    def calculate_route(self):
        """Create a route vom `actor` to `goal`.

        NOTE: Taken from
        https://github.com/carla-simulator/ros-bridge/blob/master/carla_waypoint_publisher/src/carla_waypoint_publisher/carla_waypoint_publisher.py

        Don't know, why the function does not work from Repo, but in the end,
        main functionality can be salvaged :)
        """
        self.loginfo(f'Global route planner ...')
        grp = GlobalRoutePlanner(self.world.get_map(), sampling_resolution=1)
        self.current_route = grp.trace_route(
            self.actor.get_location(),
            self.goal
        )
        self.loginfo(
            f'▹ Calculated route contains {len(self.current_route)} waypoints'
        )

        self.nodes = CarlaAPI.convert_waypoints_to_array(self.current_route)

        # Output spawnpoint of route for respawning, for random spawn points
        os.system(f'echo "{self.current_route[0][0].transform}" '
                  f'> ./log/PLANNER_SPAWN_POINT_{self.role_name}')

    def _callback_reference(self):
        roscomp.loginfo("Publish `reference` topic: ")
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = roscomp.ros_timestamp(
            self.get_time(), from_sec=True
        )

        reference_wp_indices = self.calculate_reference()

        for idx in reference_wp_indices:
            pose = PoseStamped()
            pose.pose = trans.carla_transform_to_ros_pose(
                self.current_route[idx][0].transform
            )
            msg.poses.append(pose)

        self.pub_reference.publish(msg)

    def calculate_reference(self):
        loc = self.actor.get_location()
        ego_pos = np.array([[loc.x, loc.y]])

        dist_2 = np.sum((self.nodes - ego_pos)**2, axis=1)
        idx = np.argmin(dist_2)

        # Make sure to start reference behind current position
        idx -= min(idx, 3)

        s = 0
        reference_indices = [idx]
        while s <= self.REFERENCE_MIN_LENGTH:
            idx += 1
            dx = self.nodes[idx, 0] - self.nodes[idx-1, 0]
            dy = self.nodes[idx, 1] - self.nodes[idx-1, 1]
            s += np.sqrt(dx*dx + dy*dy)
            reference_indices.append(idx)

        # CarlaAPI.draw_debug_line(
        #     points=self.nodes[reference_indices[0]:reference_indices[-1], :],
        #     world=self.world,
        #     life_time=1.5,
        #     location_z=0.1,
        #     color=carla.Color(1, 1, 1, 100)
        # )
        # self.loginfo(
        #     f'▹ Reference visualized as line in Carla (1s life time)'
        # )

        return reference_indices


def main(args=None):
    """Start basic ROS2 main function `planner` node."""
    roscomp.init('planner', args)

    planner = None
    try:
        planner = Planner()
        planner.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if planner:
            planner.destroy()
        roscomp.shutdown()


if __name__ == '__main__':
    main()
