import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class EffortControllerPublisher(Node):
    def __init__(self):
        super().__init__('effort_controller_publisher')
        # Create a publisher for the /effort_controller/commands topic
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.publish_torque_values)  # Publish every 0.5 seconds

    def publish_torque_values(self):
        msg = Float64MultiArray()
        # Example torque values (adjust as needed for your application)
        msg.data = [1000.0, 1000.0, 1000.0]  # Replace with the desired torque values
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published torque values: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = EffortControllerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
