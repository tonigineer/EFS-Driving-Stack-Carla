import carla
import numpy as np

from simple_pid import PID

import rclpy

from nav_msgs.msg import Odometry

from ros_compatibility import loginfo, logwarn
from ros_compatibility.exceptions import ROSException

from carla_efs_api import CarlaAPI
from carla_efs_messages.msg import VehicleControl, VehicleState, VehiclePhysics


class VehicleControlInterface(rclpy.node.Node):

    REFRESH_RATE_VCI_HZ = 50
    REFRESH_RATE_STATE_HZ = 50
    REFRESH_RATE_PHYSICS_HZ = 1

    TIMEOUT_VEHICLE_CONTROL = 0.5

    odometry = None

    vehicle_control = None
    vehicle_physics = None
    vehicle_state = None

    receiving = True

    def __init__(self):
        super().__init__('carla_efs_vehicle_control_interface')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.timer_vci = self.create_timer(
            1.0/self.REFRESH_RATE_VCI_HZ,
            self.callback_vehicle_control_interface
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

    def configure_subscriber(self) -> None:
        self.sub_vehicle_control = self.create_subscription(
            VehicleControl, f'/carla/{self.role_name}/vehicle_control',
            self.callback_vehicle_control, 10
        )

        self.sub_odoemtry = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self.callback_odometry, 10
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

    def callback_vehicle_control_interface(self):
        # Timeout handling
        # if not self.vehicle_control or not self.odometry:
        #     if self.receiving:
        #         logwarn(f'Waiting for /carla/{self.role_name}/vehicle_control')
        #         self.receiving = False
        #     return

        # # TODO: There must be a better solution to compare timestamps, ffs.
        # current_time = self.get_clock().now().to_msg().sec + \
        #     self.get_clock().now().to_msg().nanosec * 1e-9
        # last_received = self.vehicle_control.header.stamp.sec + \
        #     self.vehicle_control.header.stamp.nanosec * 1e-9

        # if (current_time - last_received) > self.TIMEOUT_VEHICLE_CONTROL:
        #     if self.receiving:
        #         logwarn(f'TIMEOUT:  /carla/{self.role_name}/vehicle_control')
        #         self.emergency_stop()
        #     self.receiving = False
        #     return

        # if not self.receiving:
        #     self.receiving = True
        #     loginfo(f'RECEIVED: /carla/{self.role_name}/vehicle_control')

        if not self.vehicle_control or not self.odometry or not self.vehicle_physics:
            return

        self.speed_controller.setpoint = self.vehicle_control.desired_velocity
        # loginfo(f'{self.odometry.twist.twist.linear.x}')
        speed_control = float(
            self.speed_controller(self.odometry.twist.twist.linear.x)
        )
        throttle = max(0, speed_control)
        brake = min(speed_control, 0)

        steer = -self.vehicle_control.desired_steering_angle / \
            self.vehicle_physics.max_steering_angle

        self.actor.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer
            )
        )

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
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if veh_ctrl_interface:
            veh_ctrl_interface.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
