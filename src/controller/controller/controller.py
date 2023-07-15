import carla
import numpy as np
import casadi as cs
import casadi.tools as ct
from time import perf_counter, sleep

from planner import CarlaAPI

from transforms3d.euler import quat2euler
from tf_transformations import quaternion_from_euler

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from ros_compatibility.exceptions import ROSException

from ackermann_msgs.msg import AckermannDrive


x = ct.struct_symMX(['pos_x', 'pos_y', 'psi', 'vx', 'vy', 'psip'])
u = ct.struct_symMX(['delta_v', 'ax'])


class MPCNode(CompatibleNode):

    SAMPLE_TIME_SECONDS = 0.1
    TIME_HORIZON = 3.0

    state = np.zeros([6, 1])
    reference = np.zeros([
        int(TIME_HORIZON/SAMPLE_TIME_SECONDS) + 2, 2
    ])

    def __init__(self):
        super().__init__('mpc')
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.set_up_communication()

        self.timer_main = self.create_timer(
            self.SAMPLE_TIME_SECONDS, self.update_controller
        )

        self.mpc = KinematicMPCTracking(
            step_time=self.SAMPLE_TIME_SECONDS,
            time_horizon=self.TIME_HORIZON
        )

        self.world = CarlaAPI.get_world()

    def set_up_communication(self):
        # // SUBSCRIBER
        self.sub_odom = self.create_subscription(
            Odometry, f'/carla/{self.role_name}/odometry',
            self._callback_odometry, 10
        )

        self.sub_reference = self.create_subscription(
            Path, f'/carla/{self.role_name}/planner/reference',
            self._callback_reference, 10
        )

        # // PUBLISHER
        self.pub_ackermann = self.create_publisher(
            AckermannDrive, f'/carla/{self.role_name}/ackermann_cmd', 10
        )

    def _callback_odometry(self, msg):
        _, _, yaw = quat2euler([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        state = np.array([[
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        ]]).T
        self.state = state

        # self.loginfo(f'RECEIVED ODOMETRY')

    def _callback_reference(self, msg):
        self.reference = np.array([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ])
        # self.loginfo(f'{nodes}')
        # self.reference = np.unique(nodes, axis=0)
        # self.loginfo(f'{self.state[0:2, 0]}')
        # self.loginfo(f'{nodes}')

        # _, idc = np.unique(nodes, axis=0, return_index=True)
        # nodes = nodes[sorted(idc), :]  # FFS, np.unique also sorts

        # self.loginfo(f'RECEIVED ODOMETRY')
        # self.loginfo(f'STATE: {self.state[0:2, 0]}')
        # self.loginfo(f'REF:   {nodes}')

        # self.loginfo(f'{nodes}')

        # self.state = np.array([
        #     self.vehicle.get_transform().location.x,
        #     self.vehicle.get_transform().location.y
        # ])

        # v = self.vehicle.get_velocity()
        # a = self.vehicle.get_acceleration()
        # return np.array([[
        #     self.vehicle.get_transform().location.x,
        #     self.vehicle.get_transform().location.y,
        #     self.vehicle.get_transform().rotation.yaw/180*np.pi-np.pi/2,
        #     self.vehicle.get_angular_velocity().z/180*np.pi,  # supposed to be rad/s, but value does make any sense without deg2rad
        #     np.sqrt(v.x**2 + v.y**2 + v.z**2),
        #     0
        # ]]).T

    def update_controller(self):
        x0 = self.state
        ref = self.reference[0:self.mpc.N_hor+1, :].T

        ref, ey = self.create_time_discrete_traj(
            nodes=ref,
            state=x0,
        )

        self.loginfo(f'STATE: {x0[0:2, 0]}')
        self.loginfo(f'REF:   {ref[0:2, 0]}')

        start = perf_counter()
        try:
            control_out, state_horizon = self.mpc.solve(
                state_vector=x0, reference=ref)
        except Exception as e:
            self.loginfo(f'{e}')

        msg = AckermannDrive()
        msg.steering_angle = control_out[0][0]
        msg.acceleration = 0.0 if self.state[3] > 1.5 else control_out[1][0]
        msg.speed = state_horizon[3, 1]
        self.pub_ackermann.publish(msg)

        # TODO: WTF, y-axis different to odom and wp's?
        CarlaAPI.draw_debug_line(
            points=state_horizon[0:2, :].T * [1, -1],
            world=self.world,
            life_time=0.15,
            location_z=0.1,
            thickness=0.25,
            color=carla.Color(1, 1, 1, 100)
        )

    # Time discrete reference
    @staticmethod
    def localize_on_reference(nodes, position, nrn=5) -> float:
        x = nodes[:, 0]
        y = nodes[:, 1]
        s = nodes[:, 2]

        pp = position

        # Allocate memory
        pcs = np.zeros([nrn, 2])
        pc_diffs = np.zeros([nrn, 1])
        distances = np.zeros([nrn, 1])
        in_between = np.zeros([nrn, 1])

        for i in range(nrn):
            # Points of line
            pa = np.array([x[i], y[i]])
            pb = np.array([x[i+1], y[i+1]])

            # Vectors
            AB = pb-pa
            AP = pp-pa

            # Length from pa to projection pc on AB, where vector through pp is
            # perpendicular on AB. May be outside of AB!
            pc_diff = np.dot(AB, AP) / np.dot(AB, AB) * AB
            pc = pa + pc_diff

            AC = pc-pa

            # Source: https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
            on_vector = not (np.dot(AB, AC) < 0 or np.dot(AB, AC) > np.dot(AB, AB))

            pcs[i, :] = pc
            pc_diffs[i, :] = np.linalg.norm(pc-pa)
            distances[i, :] = np.linalg.norm(pc-pp)
            in_between[i, :] = on_vector

        if np.any(in_between):
            distances[in_between == 0] = np.inf
            k = np.argmin(distances)
        else:
            k = np.argmin(pc_diffs)

        return s[k] + pc_diffs[k, 0]

    @staticmethod
    def make_time_discrete(nodes, ref, v, dt, T):
        d = ref[2]
        t = 0
        traj = np.zeros([int(T/dt) + 1, 3])
        traj[0, 0:2] = ref[0:2]
        i = 1
        while t < T:
            ds = v * dt
            t += dt
            d += ds
            traj[i, 0] = np.interp(d, nodes[:, 2], nodes[:, 0])
            traj[i, 1] = np.interp(d, nodes[:, 2], nodes[:, 1])
            traj[i, 2] = d
            i += 1
        return traj

    def create_time_discrete_traj(self, nodes, state):
        _, idc = np.unique(nodes, axis=1, return_index=True)

        # No reference received yet, just zeros
        if len(idc) == 1:
            self.loginfo(f'Time discrete trajectory cannot be calculated')
            return nodes, 0

        nodes = self.reference[sorted(idc), 0:2]
        position = state[0:2, 0].T
        vx = state[3, 0]

        s = np.zeros([nodes.shape[0], 1])
        d = 0
        for i in range(1, nodes.shape[0]):
            dx = nodes[i, 0]-nodes[i-1, 0]
            dy = nodes[i, 1]-nodes[i-1, 1]
            d += np.sqrt(dx*dx + dy*dy)
            s[i] = d

        nodes = np.hstack([nodes, s])

        s_ref = self.localize_on_reference(nodes, position)

        x_ref = np.interp(s_ref, nodes[:, 2], nodes[:, 0])
        y_ref = np.interp(s_ref, nodes[:, 2], nodes[:, 1])

        e_y = np.linalg.norm(
            np.array([x_ref-position[0], y_ref-position[1]])
        )

        traj = self.make_time_discrete(
            nodes,
            np.array([x_ref, y_ref, s_ref]),
            v=2, dt=self.SAMPLE_TIME_SECONDS, T=self.TIME_HORIZON+self.SAMPLE_TIME_SECONDS
        )

        return traj[:, 0:2].T, e_y


class KinematicMPCTracking:
    """A CasADi based MPC for trajectory tracking."""

    def __init__(self, step_time: float = 0.1,
                 time_horizon: float = 2.0) -> None:
        self.dt = step_time
        self.T_hor = time_horizon
        self.N_hor = int(time_horizon / step_time) + 1

        self.define_nominal_model()
        self.dxdt = self.integration(self.f_nominal)

        self.formulate_ocp()
        self.configure_solver()

    def define_nominal_model(self) -> cs.Function:
        """Nominal model of rover to calculate global movement.

        NOTE: Model is bullshit for a rover :) It's basically
        a simple kinematic bicycle, but the yaw motion can be
        given instead of resulting from a steering input.
        """
        vxp = u['ax']
        vyp = 0  # (u['vy_cmd'] - x['vy']) / self.dt
        psipp = 0  # (u['psip_cmd'] - x['psip']) / self.dt

        L = 2.928

        vx = x['vx'] * cs.cos(x['psi'])
        vy = x['vx'] * cs.sin(x['psi'])
        psip = x['vx'] * np.tan(u['delta_v']) / L

        rhs = cs.vertcat(vx, vy, psip, vxp, vyp, psipp)
        self.f_nominal = ct.Function('f', [x, u], [rhs], ['x', 'u'], ['dx/dt'])

    def integration(self, f) -> cs.Function:
        """Define Runge-Kutta 4 integrator.

        NOTE: CasADi uses SUNIDALS integrator, which comes
        with an additional overhead. Therefore, lightweight
        implementation is used to save computational costs.
        """
        k1 = f(x, u)
        k2 = f(x + self.dt / 2.0 * k1, u)
        k3 = f(x + self.dt / 2.0 * k2, u)
        k4 = f(x + self.dt * k3, u)
        xf = x + self.dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

        return cs.Function(
            "F_RK4", [x, u], [xf], ['x[k]', 'u[k]'], ['x[k+1]']
        )

    def formulate_ocp(self):
        """Define optimal control problem."""
        opti = cs.Opti()

        # Decision variables for states and inputs
        X = opti.variable(x.size, self.N_hor+1)
        U = opti.variable(u.size, self.N_hor)

        # Initialize parameter
        x0 = opti.parameter(x.size)
        ref = opti.parameter(2, self.N_hor+1)

        # Build iteration over time horizon
        for k in range(self.N_hor):
            opti.subject_to(X[:, k+1] == self.dxdt(X[:, k], U[:, k]))

        opti.subject_to(X[:, 0] == x0)

        # Readability improvement, maybe for people who don't know the code?
        delta_v, ax = U[0, :], U[1, :]
        pos_x, pos_y, vx = X[0, :], X[1, :], X[3, :]
        pos_x_ref, pos_y_ref = ref[0, :], ref[1, :]

        # Constraints
        delta_v_max = np.deg2rad(50)
        opti.subject_to(opti.bounded(-delta_v_max, delta_v, delta_v_max))
        opti.subject_to(opti.bounded(-7.0, ax, 3.0))

        # Cost function
        Q = np.diag([100, 100, 0, 1000, 0, 0])
        R = np.diag([10, 1])

        opti.minimize(
            cs.sumsqr(pos_x - pos_x_ref) * Q[0, 0] +
            cs.sumsqr(pos_y - pos_y_ref) * Q[1, 1] +
            cs.sumsqr(vx - 2) * Q[3, 3] +
            cs.sumsqr(delta_v) * R[0, 0] +
            cs.sumsqr(ax) * R[1, 1]
        )

        # Provide for solver
        self.opti = opti
        self.U, self.X = U, X
        self.x0, self.ref = x0, ref

    def configure_solver(self):
        """Configure solver with hardcoded settings."""
        opts = {
            'ipopt': {
                'tol': 1e-10,
                'max_iter': 200,
                'print_level': 0,  # remove ipopt disclaimer
                'sb': 'yes',       # remove ipopt disclaimer
                'acceptable_tol': 1e-2,
                'acceptable_obj_change_tol': 1e-2
            },
            'expand': True,
            'print_time': 0
        }
        self.opti.solver('ipopt', opts)

    def solve(self, state_vector: np.ndarray, reference: np.ndarray):
        """Solve OPC for initial state and reference."""
        self.opti.set_value(self.x0, state_vector)
        self.opti.set_value(self.ref, reference)
        sol = self.opti.solve()
        return sol.value(self.U).reshape(u.size, -1), sol.value(self.X)


def main(args=None):
    """Start basic ROS2 main function `planner` node."""
    roscomp.init('mpc_controller', args)

    controller = None
    try:
        controller = MPCNode()
        controller.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if controller:
            # Stop last command for Ackermann-Interface.
            # Does not really work though :(
            msg = AckermannDrive()
            msg.acceleration = -6.0
            msg.speed = 0.0
            msg.steering_angle = 0.0
            sleep(0.5)
            controller.pub_ackermann.publish(msg)

            controller.destroy()
        roscomp.shutdown()


if __name__ == '__main__':
    main()
