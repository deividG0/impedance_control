import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AdmittanceController3DOF(Node):
    def __init__(self):
        super().__init__('admittance_controller_3dof')

        # Subscribers for joint forces
        self.force_subs = [
            self.create_subscription(
                Wrench,
                f'/force_torque_joint{i+1}',
                self.create_force_callback(i),
                10
            ) for i in range(3)  # Assuming 3 DOF
        ]

        # Joint states subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for desired trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Initialize parameters
        self.mass = [1.0, 1.0, 1.0]  # Example values for each joint
        self.damping = [5.0, 5.0, 5.0]  # Example values for each joint
        self.stiffness = [10.0, 10.0, 10.0]  # Example values for each joint
        self.dt = 0.01  # Control loop timestep

        # State variables
        self.current_positions = [0.0, 0.0, 0.0]
        self.current_velocities = [0.0, 0.0, 0.0]
        self.external_forces = [0.0, 0.0, 0.0]

    def create_force_callback(self, joint_index):
        def force_callback(msg: Wrench):
            # self.get_logger().info(f"Force callback: {msg.force.x}, {type(msg.force.x)}")
            # self.external_forces[joint_index] = msg.force.x*100
            if joint_index == 2:
                self.external_forces[joint_index] = 100.0
            else:
                self.external_forces[joint_index] = 0.0
            self.compute_admittance()
        return force_callback

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 3 and len(msg.velocity) >= 3:
            self.current_positions = msg.position[:3]
            self.current_velocities = msg.velocity[:3]

    def compute_admittance(self):
        desired_positions = []
        for i in range(3):
            # Compute desired acceleration
            desired_acceleration = (
                self.external_forces[i] / self.mass[i]
                - (self.damping[i] / self.mass[i]) * self.current_velocities[i]
                - (self.stiffness[i] / self.mass[i]) * self.current_positions[i]
            )
            # Compute desired velocity and position
            desired_velocity = self.current_velocities[i] + desired_acceleration * self.dt
            desired_position = self.current_positions[i] + desired_velocity * self.dt
            desired_positions.append(desired_position)

        # Publish trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with your joint names
        point = JointTrajectoryPoint()
        point.positions = desired_positions
        point.time_from_start = rclpy.duration.Duration(seconds=self.dt).to_msg()
        traj_msg.points.append(point)

        self.trajectory_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AdmittanceController3DOF()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
