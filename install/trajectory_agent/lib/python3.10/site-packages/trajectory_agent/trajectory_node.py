import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 2.0
        msg.pose.position.y = 3.0
        msg.pose.orientation.w = 1.0  # sin rotación
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal_pose: ({msg.pose.position.x}, {msg.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
