import rclpy

from carla_efs_messages.msg import VehicleControl

from ros_compatibility import loginfo
from ros_compatibility.exceptions import ROSException


class MPCTrajTrack(rclpy.node.Node):

    REFRESH_RATE_MPC_HZ = 50

    def __init__(self):
        super().__init__('carla_efs_mpc_trajectory_tracking')
        self.declare_parameter('role_name', None)
        self.role_name = self.get_parameter('role_name').value

        self.configure_publisher()

    def configure_publisher(self) -> None:
        self.timer_mpc = self.create_timer(
            1.0/self.REFRESH_RATE_MPC_HZ,
            self.callback_mpc
        )
        self.pub_vehctrl = self.create_publisher(
            VehicleControl, f'/carla/{self.role_name}/vehicle_control', 10
        )

    def callback_mpc(self):
        loginfo('mpc update')
        msg = VehicleControl()
        msg.header.frame_id = self.role_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.desired_velocity = 10.0
        msg.desired_steering_angle = 0.2
        self.pub_vehctrl.publish(msg)

    def destroy(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    mpc = None
    try:
        mpc = MPCTrajTrack()
        rclpy.spin(mpc)
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        loginfo("User requested shut down.")
    finally:
        loginfo("Shutting down.")
        if mpc:
            mpc.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
