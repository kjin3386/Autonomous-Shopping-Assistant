import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.subscription = self.create_subscription(
            Odometry,
            '/scout_mini_base_controller/odom',
            self.odom_callback,
            10)
        self.path_publisher = self.create_publisher(Path, '/odom_path', 10)
        self.path = Path()
        self.path.header.frame_id = "odom"

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

