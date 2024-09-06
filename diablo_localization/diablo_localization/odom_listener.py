import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10)
        self.subscription

    def odom_callback(self, msg):
        self.get_logger().info(f'Position: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}')
def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)
    odom_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
