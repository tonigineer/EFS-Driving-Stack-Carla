import carla
import numpy as np
from typing import Tuple

from simple_pid import PID

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from carla_efs_api import CarlaAPI
from carla_efs_api.ros_logging import loginfo, logwarn
from carla_efs_messages.msg import VehicleControl, VehicleState, VehiclePhysics


class VehicleControlInterface(Node):

    REFRESH_RATE_VCI_HZ = 10
    REFRESH_RATE_STATE_HZ = 50
    REFRESH_RATE_PHYSICS_HZ = 1

    TIMEOUT = 0.5

    odometry = None
    imu = None

    vehicle_control = None
    vehicle_physics = None
    vehicle_state = None

    receiving = True
    last_received_odom = 0
    last_received_vehctrl = 0
    last_received_imu = 0

    # Acceleration from IMU is basically useless.
    # TODO: Checking if settings can be applied for better data.
    accel_last_time_sec = 0
    accel_last_vx = 0
    accel_memory = []

    def __init__(self):
        super().__init__('carla_efs_vehicle_control_interface')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.timer_vci = self.create_timer(
            1.0/self.REFRESH_RATE_VCI_HZ,
            self.update_vehicle_control_interface
        )

        self.world = CarlaAPI.get_world()
        self.actor = CarlaAPI.get_actors(
            world=self.world, pattern=[self.role_name]
        )[0]

        self.configure_subscriber()
        self.configure_publisher()

        self.speed_controller = PID(
            Kp=0.125, Ki=0.015, Kd=0.075,
            sample_time=1.0/self.REFRESH_RATE_VCI_HZ,
            output_limits=(-1., 1.)
        )

        self.accel_controller = PID(
            Kp=0.125, Ki=0.015, Kd=0.075,
            sample_time=1.0/self.REFRESH_RATE_VCI_HZ,
            output_limits=(-1., 1.)
        )

    def configure_subscriber(self) -> None:
        self.sub_vehicle_control = self.create_subscription(
            VehicleControl, f'/carla/{self.role_name}/vehicle_control',
            self.callback_vehicle_control, 10
        )

        self.sub_odoemtry = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self.callback_odometry, 10
        )

        self.sub_imu = self.create_subscription(
            Imu, f'/carla/{self.role_name}/imu',
            self.callback_imu, 10
        )

    def configure_publisher(self) -> None:
        self.timer_phsysics = self.create_timer(
            1.0/self.REFRESH_RATE_PHYSICS_HZ,
            self.callback_physics
        )
        self.pub_physics = self.create_publisher(
            VehiclePhysics, f'/carla/{self.role_name}/vehicle_physics', 10
        )

        self.timer_state = self.create_timer(
            1.0/self.REFRESH_RATE_STATE_HZ,
            self.callback_state
        )
        self.pub_state = self.create_publisher(
            VehicleState, f'/carla/{self.role_name}/vehicle_state', 10
        )

    def callback_odometry(self, msg):
        self.odometry = msg
        self.last_received_odom = msg.header.stamp.sec + \
            msg.header.stamp.nanosec * 1e-9

    def callback_imu(self, msg):
        self.imu = msg
        self.last_received_imu = msg.header.stamp.sec + \
            msg.header.stamp.nanosec * 1e-9

    def callback_physics(self):
        if not self.vehicle_physics:
            self.get_vehicle_physics()

        msg = self.vehicle_physics
        msg.header.frame_id = self.role_name
        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_physics.publish(msg)

    def callback_state(self):
        self.get_vehicle_info()

        self.vehicle_state.header.frame_id = self.role_name
        self.vehicle_state.header.stamp = self.get_clock().now().to_msg()

        self.pub_state.publish(self.vehicle_state)

    def callback_vehicle_control(self, msg):
        self.vehicle_control = msg
        self.last_received_vehctrl = msg.header.stamp.sec + \
            msg.header.stamp.nanosec * 1e-9

    def update_vehicle_control_interface(self):
        if not self._timeout_handling():
            return

        throttle, brake = self.longitudinal_control()
        steer = self.lateral_control()

        self.actor.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer,
                gear=2
            )
        )

    def longitudinal_control(self) -> Tuple[float, float]:

        # loginfo(f'speed ctrl: set {self.vehicle_control.desired_velocity:0.1f} current {self.odometry.twist.twist.linear.x:0.1f}')
        # self.speed_controller.setpoint = self.vehicle_control.desired_velocity
        # speed_control = float(
        #     self.speed_controller(self.odometry.twist.twist.linear.x)
        # )
        # throttle = max(0, speed_control)
        # brake = min(speed_control, 0)
        WINDOW_SIZE = 50

        current_time_sec = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        dvx = self.odometry.twist.twist.linear.x - self.accel_last_vx
        dts = current_time_sec - self.accel_last_time_sec

        accel = dvx / dts

        self.accel_memory.append(accel)
        if len(self.accel_memory) > WINDOW_SIZE:
            self.accel_memory.pop(0)

        self.accel_last_vx = self.odometry.twist.twist.linear.x
        self.accel_last_time_sec = current_time_sec

        accel = sum(self.accel_memory) / len(self.accel_memory)

        loginfo(f'Accleration control | Set point: {self.vehicle_control.desired_acceleration:+0.1f}m/ss - Vehicle: {accel:+0.1f} m/ss')

        self.accel_controller.setpoint = self.vehicle_control.desired_acceleration
        accel_control = float(
            self.accel_controller(accel)
        )
        throttle = max(0, accel_control)
        brake = -min(accel_control, 0)

        loginfo(f'{accel_control}')

        return (throttle, brake)

    def lateral_control(self) -> float:
        return -self.vehicle_control.desired_steering_angle / \
            self.vehicle_physics.max_steering_angle

    def _timeout_handling(self) -> bool:
        if not self.vehicle_physics:
            if self.receiving:
                logwarn(f'Vehicle physics not parsed yet.')
                self.receiving = False
            return False

        current_time = self.get_clock().now().to_msg().sec + \
            self.get_clock().now().to_msg().nanosec * 1e-9

        if np.abs(current_time - self.last_received_odom) > self.TIMEOUT or \
           np.abs(current_time - self.last_received_vehctrl) > self.TIMEOUT or \
           np.abs(current_time - self.last_received_imu) > self.TIMEOUT:
            if self.receiving:
                logwarn(
                    f'Timeout - {"/odometry" if np.abs(current_time - self.last_received_odom) <= self.TIMEOUT else ""} '
                    f'{"/vehicle_control" if np.abs(current_time - self.last_received_vehctrl) <= self.TIMEOUT else ""}'
                )
                self.emergency_stop()
                self.receiving = False
            return False

        if not self.receiving:
            loginfo(
                f'Receiving - {"/odometry" if np.abs(current_time - self.last_received_odom) <= self.TIMEOUT else ""} '
                f'{"/vehicle_control" if np.abs(current_time - self.last_received_vehctrl) <= self.TIMEOUT else ""}'
            )
            self.receiving = True

        return True

    def get_vehicle_info(self):
        if not self.vehicle_state:
            self.vehicle_state = VehicleState()

        self.vehicle_state.throttle_position = self.actor.get_control().throttle
        self.vehicle_state.brake_position = self.actor.get_control().brake
        self.vehicle_state.steering_position = self.actor.get_control().steer
        self.vehicle_state.gear = self.actor.get_control().gear

    def get_vehicle_physics(self):
        physics_control = self.actor.get_physics_control()

        max_steering_angle_deg = physics_control.wheels[0].max_steer_angle
        assert physics_control.wheels[0].max_steer_angle == \
            physics_control.wheels[1].max_steer_angle, \
            f'Front tires have different max steering angle!'

        msg = VehiclePhysics()
        msg.max_steering_angle = np.deg2rad(max_steering_angle_deg)
        self.vehicle_physics = msg

    def emergency_stop(self):
        self.actor.apply_control(
            carla.VehicleControl(
                throttle=0,
                brake=1,
                steer=0
            )
        )

    def destroy(self):
        self.emergency_stop()


def main(args=None):
    rclpy.init(args=args)

    veh_ctrl_interface = None
    try:
        veh_ctrl_interface = VehicleControlInterface()
        rclpy.spin(veh_ctrl_interface)
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if veh_ctrl_interface:
            veh_ctrl_interface.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
