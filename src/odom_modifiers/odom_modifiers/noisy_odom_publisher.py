import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random
from scipy.spatial.transform import Rotation as R

class NoisyOdomPublisher(Node):
    def __init__(self):
        super().__init__('noisy_odom_publisher')

        self.sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/ego_racecar/odom_noisy', 10)

        self.pos_noise_std = 0.05  # metros
        self.yaw_noise_std = 0.02  # radianes

    def odom_callback(self, msg):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id
        noisy_msg.twist = msg.twist
        noisy_msg.pose.pose = msg.pose.pose

        noisy_msg.pose.pose.position.x += random.gauss(0, self.pos_noise_std)
        noisy_msg.pose.pose.position.y += random.gauss(0, self.pos_noise_std)

        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz')
        noisy_yaw = yaw + random.gauss(0, self.yaw_noise_std)
        new_q = R.from_euler('xyz', [roll, pitch, noisy_yaw]).as_quat()

        noisy_msg.pose.pose.orientation.x = new_q[0]
        noisy_msg.pose.pose.orientation.y = new_q[1]
        noisy_msg.pose.pose.orientation.z = new_q[2]
        noisy_msg.pose.pose.orientation.w = new_q[3]

        self.pub.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

