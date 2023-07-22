from random import choice

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from carla_efs_api import CarlaAPI, loginfo
from carla_efs_api import Transformations as tf


class Spawner(Node):

    ODOMETRY_REFRESH_RATE_HZ = 200

    actor = None

    def __init__(self):
        super().__init__('carla_efs_spawner')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.world = CarlaAPI.get_world()

        self.configure_subscriber()
        self.configure_publisher()

        self.spawn_vehicle()
        CarlaAPI.move_to_actor(world=self.world, pattern=[self.role_name])

    def callback_odometry(self) -> None:
        msg = Odometry()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = self.role_name

        msg.pose.pose = tf.carla_transform_to_ros_pose(
            self.actor.get_transform()
        )

        msg.twist.twist = tf.carla_velocity_to_ros_twist(
            self.actor.get_velocity(),
            self.actor.get_angular_velocity(),
            self.actor.get_transform().rotation)

        self.pub_odometry.publish(msg)

    def configure_subscriber(self) -> None:
        pass

    def configure_publisher(self) -> None:
        self.pub_odometry = self.create_publisher(
            Odometry, f'/carla/{self.role_name}/odometry', 10
        )
        self.timer_pub_odometry = self.create_timer(
            1.0/self.ODOMETRY_REFRESH_RATE_HZ,
            self.callback_odometry
        )

    def spawn_vehicle(self):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.audi.etron')[0]
        vehicle_bp.set_attribute('role_name', self.role_name)

        transform = choice(self.world.get_map().get_spawn_points())
        self.actor = self.world.spawn_actor(
            vehicle_bp,
            transform,
        )

        self.actor.set_simulate_physics(True)

        loginfo(
            f'{self.actor.type_id.upper()} {self.actor.id} '
            f'({self.role_name}) spawned at {transform.location}'
        )

    def destroy(self):
        loginfo(
            f'{self.actor.type_id.upper()} {self.actor.id}'
            f'({self.role_name}) was destroyed.'
        )
        self.actor.destroy()


def main(args=None):
    rclpy.init(args=args)

    try:
        spawner = Spawner()
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if spawner:
            spawner.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
