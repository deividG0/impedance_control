import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class ImpedanceControlNode(Node):
    def __init__(self):
        super().__init__('impedance_control_node')

        # Publisher for joint efforts
        self.effort_pub = self.create_publisher(Float64MultiArray, '/joint_efforts', 10)

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Impedance parameters
        self.stiffness = np.array([10.0, 10.0, 10.0])  # Nm/rad
        self.damping = np.array([1.0, 1.0, 1.0])       # Nms/rad

        # Desired positions
        self.desired_positions = np.array([0.0, 0.0, 0.0])

    def joint_state_callback(self, msg):
        positions = np.array(msg.position)
        velocities = np.array(msg.velocity)

        # Impedance control law: τ = K*(qd - q) - D*q_dot
        efforts = self.stiffness * (self.desired_positions - positions) - self.damping * velocities

        # Publish efforts
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts.tolist()
        self.effort_pub.publish(effort_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
