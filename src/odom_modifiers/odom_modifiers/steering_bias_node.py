import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class SteeringBiasNode(Node):
    def __init__(self):
        super().__init__('steering_bias_node')
        self.offset = 0.1  # radianes
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive_raw',
            self.drive_callback,
            10
        )
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        self.get_logger().info('Steering bias node initialized')

    def drive_callback(self, msg):
        msg.drive.steering_angle += self.offset
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringBiasNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
