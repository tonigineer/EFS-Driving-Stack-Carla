import carla
from random import choice

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from transforms3d.euler import euler2quat

from carla_efs_api import CarlaAPI
import carla_efs_api.transformations as tf
from carla_efs_api.ros_logging import loginfo


class Spawner(Node):

    PUBLISHER_REFRESH_RATE_HZ = 50

    actor = None
    imu = None
    gnss = None

    def __init__(self):
        super().__init__('carla_efs_spawner')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.declare_parameter('imu_attached', True)
        self.imu_attached = self.get_parameter('imu_attached').value

        self.declare_parameter('gnss_attached', True)
        self.gnss_attached = self.get_parameter('gnss_attached').value

        self.world = CarlaAPI.get_world()

        self.configure_subscriber()
        self.configure_publisher()

        self.spawn_vehicle()
        self.spawn_sensory()

        CarlaAPI.move_to_actor(world=self.world, pattern=[self.role_name])

    def callback_publisher(self) -> None:
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

        if self.imu_attached:
            msg = Imu()

            msg.header.frame_id = self.role_name
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.angular_velocity.x = -self.imu_data.gyroscope.x
            msg.angular_velocity.y = self.imu_data.gyroscope.y
            msg.angular_velocity.z = -self.imu_data.gyroscope.z

            msg.linear_acceleration.x = self.imu_data.accelerometer.x
            msg.linear_acceleration.y = -self.imu_data.accelerometer.y
            msg.linear_acceleration.z = self.imu_data.accelerometer.z

            roll, pitch, yaw = tf.carla_rotation_to_RPY(
                self.imu_data.transform.rotation
            )
            quat = euler2quat(roll, pitch, yaw)
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            self.pub_imu.publish(msg)

        if self.gnss_attached:
            msg = NavSatFix()

            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.latitude = self.gnss_data.latitude
            msg.longitude = self.gnss_data.longitude
            msg.altitude = self.gnss_data.altitude

            self.pub_gnss.publish(msg)

    def callback_imu(self, data):
        self.imu_data = data

    def callback_gnss(self, data):
        self.gnss_data = data

    def configure_subscriber(self) -> None:
        pass

    def configure_publisher(self) -> None:
        self.pub_odometry = self.create_publisher(
            Odometry, f'/carla/{self.role_name}/odometry', 10
        )

        if self.imu_attached:
            self.pub_imu = self.create_publisher(
                Imu, f'/carla/{self.role_name}/imu', 10
            )
        if self.gnss_attached:
            self.pub_gnss = self.create_publisher(
                NavSatFix, f'/carla/{self.role_name}/gnss', 10
            )

        self.timer_publisher = self.create_timer(
            1.0/self.PUBLISHER_REFRESH_RATE_HZ,
            self.callback_publisher
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

    def spawn_sensory(self):
        if self.imu_attached:
            bp = self.world.get_blueprint_library().find('sensor.other.imu')
            self.imu = self.world.spawn_actor(
                bp, carla.Transform(),
                attach_to=self.actor
            )

            self.imu.listen(
                lambda data: self.callback_imu(data)
            )

            loginfo('ᒻIMU attached.')

        if self.gnss_attached:
            bp = self.world.get_blueprint_library().find('sensor.other.gnss')
            self.gnss = self.world.spawn_actor(
                bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
                attach_to=self.actor
            )

            self.gnss.listen(
                lambda data: self.callback_gnss(data)
            )

            loginfo('ᒻGNSS attached.')

    def destroy(self):
        loginfo(
            f'{self.actor.type_id.upper()} {self.actor.id}'
            f'({self.role_name}) was destroyed.'
        )
        self.actor.destroy()
        if self.imu:
            loginfo('ᒻattached IMU was destroyed.')
            self.imu.destroy()
        if self.gnss:
            self.gnss.destroy()
            loginfo('ᒻattached GNSS was destroyed.')


def main(args=None):
    rclpy.init(args=args)

    spawner = None
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
