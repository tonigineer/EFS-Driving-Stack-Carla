import carla
import math
import numpy as np

from typing import Tuple

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from transforms3d.euler import euler2mat, euler2quat


# SOURCE: https://github.com/carla-simulator/ros-bridge/blob/master/carla_common/src/carla_common/transforms.py


def carla_location_to_ros_point(carla_location: carla.Location) -> Point:
    """Convert a carla location to a ROS point.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point


def carla_rotation_to_ros_quaternion(
        carla_rotation: carla.Rotation) -> Quaternion:
    """Convert a carla rotation to a ROS quaternion.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = euler2quat(roll, pitch, yaw)
    ros_quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
    return ros_quaternion


def carla_rotation_to_RPY(
        carla_rotation: carla.Rotation) -> Tuple[float, float, float]:
    """Convert a carla rotation to a roll, pitch, yaw tuple.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)


def carla_transform_to_ros_pose(carla_transform: carla.Transform) -> Pose:
    """Convert carla transform to ROS pose.

    Details regarding conventions are noted within the subfunctions.
    """
    ros_pose = Pose()

    ros_pose.position = carla_location_to_ros_point(
        carla_transform.location)
    ros_pose.orientation = carla_rotation_to_ros_quaternion(
        carla_transform.rotation)

    return ros_pose


def carla_velocity_to_ros_twist(
        carla_linear_velocity: carla.libcarla.Vector3D,
        carla_angular_velocity: carla.libcarla.Vector3D,
        carla_rotation: carla.Rotation = None) -> Twist:
    """Convert a carla velocity to a ROS twist.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    """
    ros_twist = Twist()
    if carla_rotation:
        ros_twist.linear = carla_vector_to_ros_vector_rotated(
            carla_linear_velocity,
            carla_rotation
        )
    else:
        ros_twist.linear = carla_location_to_ros_vector3(carla_linear_velocity)
    ros_twist.angular.x = math.radians(carla_angular_velocity.x)
    ros_twist.angular.y = -math.radians(carla_angular_velocity.y)
    ros_twist.angular.z = -math.radians(carla_angular_velocity.z)
    return ros_twist


def carla_vector_to_ros_vector_rotated(
        carla_vector, carla_rotation: carla.Rotation) -> Vector3:
    """Rotate carla vector, return it as ros vector."""
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    tmp_array = rotation_matrix.dot(
        np.array([carla_vector.x, carla_vector.y, carla_vector.z])
    )
    ros_vector = Vector3()
    ros_vector.x = tmp_array[0]
    ros_vector.y = -tmp_array[1]
    ros_vector.z = tmp_array[2]
    return ros_vector


def carla_location_to_ros_vector3(carla_location: carla.Location) -> Vector3:
    """Convert a carla location to a ROS vector3.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    """
    ros_translation = Vector3()
    ros_translation.x = carla_location.x
    ros_translation.y = -carla_location.y
    ros_translation.z = carla_location.z

    return ros_translation


def carla_rotation_to_numpy_rotation_matrix(
        carla_rotation: carla.Rotation) -> np.ndarray:
    """Convert a carla rotation to a ROS quaternion.

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = euler2mat(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix
