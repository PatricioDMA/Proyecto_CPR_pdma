import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.publisher_ = self.create_publisher(Path, '/planned_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.trajectory = self.generate_trajectory()

    def generate_trajectory(self):
        path = Path()
        path.header.frame_id = "map"

        start_x = 0.0
        start_y = 0.0
        step = 0.2

        def add_pose(x, y):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # 1. Hacia arriba (x+)
        for i in range(50):
            add_pose(start_x + i * step, start_y)

        # 2. Hacia la izquierda (y+)
        for i in range(45):
            add_pose(start_x + (50 - 1) * step, start_y + i * step)

        # 3. Hacia abajo (x-)
        for i in range(120):
            add_pose(start_x + (50 - 1 - i) * step, start_y + (45 - 1) * step)

        # 4. Hacia la derecha (y-)
        for i in range(45):
            add_pose(start_x - 70 * step, start_y + (45 - 1 - i) * step)

        # 5. Hacia arriba (x+)
        for i in range(65):
            add_pose(start_x - (70 - i) * step, start_y)

        return path

    def timer_callback(self):
        self.trajectory.header.stamp = self.get_clock().now().to_msg()
        for pose in self.trajectory.poses:
            pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.trajectory)
        self.get_logger().info(f'Published trajectory with {len(self.trajectory.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
