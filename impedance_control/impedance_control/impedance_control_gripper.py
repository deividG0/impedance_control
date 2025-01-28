import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ImpedanceControlNode(Node):
    def __init__(self):
        super().__init__('impedance_control_node')

        # Declare parameters
        self.declare_parameter('k', 50.0)  # Stiffness coefficient
        self.declare_parameter('d', 10.0)  # Damping coefficient

        # Get parameters
        self.k = self.get_parameter('k').value
        self.d = self.get_parameter('d').value

        # Initialize state variables
        self.desired_position = 0.0
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.last_position = 0.0
        self.effort_command = 0.0

        # Create subscribers and publishers
        self.position_sub = self.create_subscription(
            Float64,
            '/desired_position',
            self.desired_position_callback,
            10
        )
        self.current_position_sub = self.create_subscription(
            Float64,
            '/current_position',
            self.current_position_callback,
            10
        )
        self.effort_pub = self.create_publisher(Float64, '/effort_command', 10)

        # Create a timer for the control loop
        self.timer = self.create_timer(0.01, self.control_loop_callback)  # 100 Hz
        self.get_logger().info('Impedance Control Node Initialized')

    def desired_position_callback(self, msg):
        """Callback to update the desired position."""
        self.desired_position = msg.data

    def current_position_callback(self, msg):
        """Callback to update the current position."""
        self.last_position = self.current_position
        self.current_position = msg.data

        # Estimate velocity using finite difference
        self.current_velocity = (self.current_position - self.last_position) / 0.01

    def control_loop_callback(self):
        """Main control loop for impedance control."""
        # Calculate position error
        position_error = self.desired_position - self.current_position

        # Impedance control law: F = k * error - d * velocity
        self.effort_command = self.k * position_error - self.d * self.current_velocity

        # Publish the effort command
        effort_msg = Float64()
        effort_msg.data = self.effort_command
        self.effort_pub.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
