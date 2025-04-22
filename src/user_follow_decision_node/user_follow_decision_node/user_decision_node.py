import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

class UserDecisionNode(Node):
    def __init__(self):
        super().__init__('user_decision_node')
        self.sub_distance = self.create_subscription(Float32, '/user_distance', self.dist_callback, 10)
        self.sub_obstacle = self.create_subscription(Bool, '/obstacle_detected', self.obs_callback, 10)
        self.command_pub = self.create_publisher(String, '/movement_command', 10)
        self.user_distance = None
        self.obstacle = False

    def dist_callback(self, msg):
        self.user_distance = msg.data
        self.make_decision()

    def obs_callback(self, msg):
        self.obstacle = msg.data
        self.make_decision()

    def make_decision(self):
        if self.user_distance is None:
            return
        if self.obstacle or self.user_distance > 2.0:
            self.command_pub.publish(String(data='STOP'))
        else:
            self.command_pub.publish(String(data='FOLLOW'))

def main(args=None):
    rclpy.init(args=args)
    node = UserDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

