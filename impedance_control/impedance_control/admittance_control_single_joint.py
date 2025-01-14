import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        # Subscribers
        self.force_sub = self.create_subscription(
            Wrench,
            '/force_torque_joint1',
            self.force_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Initialize parameters
        self.mass = 1.0  # Example value
        self.damping = 5.0  # Example value
        self.stiffness = 10.0  # Example value
        self.dt = 0.01  # Control loop timestep

        # State variables
        self.current_position = [0.0]
        self.current_velocity = [0.0]
        self.external_force = 0.5

    def force_callback(self, msg: Wrench):
        self.external_force = msg.force.z
        self.get_logger().info(f"Force callback: {msg.force}")
        self.compute_admittance()

    def joint_state_callback(self, msg: JointState):
        # self.get_logger().info(f"Joint 1 state callback: {msg}")
        if len(msg.position) > 0 and len(msg.velocity) > 0:
            self.current_position = [msg.position[2]]
            self.current_velocity = [msg.velocity[2]]

    def compute_admittance(self):
        # Compute desired acceleration
        desired_acceleration = (
            self.external_force / self.mass
            - (self.damping / self.mass) * self.current_velocity[0]
            - (self.stiffness / self.mass) * self.current_position[0]
        )
        # Compute desired velocity and position
        desired_velocity = self.current_velocity[0] + desired_acceleration * self.dt
        desired_position = self.current_position[0] + desired_velocity * self.dt

        # Publish trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with your joint names
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, desired_position]
        point.time_from_start = rclpy.duration.Duration(seconds=self.dt).to_msg()
        traj_msg.points.append(point)

        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f"Traj sent! {traj_msg}")

def main(args=None):
    rclpy.init(args=args)
    controller = AdmittanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
