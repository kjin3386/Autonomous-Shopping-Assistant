import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.sub_cmd = self.create_subscription(String, '/movement_command', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cmd_callback(self, msg):
        twist = Twist()
        if msg.data == 'FOLLOW':
            twist.linear.x = 0.3
        else:
            twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

