import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Ensure this is the correct import for publishing odometry
from motion_msgs.msg import LegMotors  # Adjust this to the correct import path
import math
from tf_transformations import quaternion_from_euler  # Import to convert theta to quaternion

class DiabloOdometry(Node):
    def __init__(self):
        super().__init__('diablo_odometry')
        self.subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.listener_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'diablo/odometry', 10)

        self.robot = Robot()  # Initialize the robot instance here

        # Initialize previous wheel positions
        self.prev_left_wheel_pos = None
        self.prev_right_wheel_pos = None

    def listener_callback(self, msg):
        left_wheel_enc_rev = msg.left_wheel_enc_rev
        right_wheel_enc_rev = msg.right_wheel_enc_rev

        # Calculate the current positions based on the encoder readings
        current_left_wheel_pos = msg.left_wheel_pos + left_wheel_enc_rev * 2 * math.pi
        current_right_wheel_pos = msg.right_wheel_pos + right_wheel_enc_rev * 2 * math.pi

        # Initialize previous positions on first callback
        if self.prev_left_wheel_pos is None or self.prev_right_wheel_pos is None:
            self.prev_left_wheel_pos = current_left_wheel_pos
            self.prev_right_wheel_pos = current_right_wheel_pos
            return  # Skip the rest of the callback to avoid erroneous initial movement

        # Calculate the change in wheel positions
        delta_left_wheel_pos = current_left_wheel_pos - self.prev_left_wheel_pos
        delta_right_wheel_pos = current_right_wheel_pos - self.prev_right_wheel_pos

        # Update the previous positions
        self.prev_left_wheel_pos = current_left_wheel_pos
        self.prev_right_wheel_pos = current_right_wheel_pos

        # Update odometry based on the change in encoder positions
        self.robot.updateOdometry(delta_left_wheel_pos, delta_right_wheel_pos)

        # Publish the odometry information
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Set position
        odom_msg.pose.pose.position.x = self.robot.Pose.x
        odom_msg.pose.pose.position.y = self.robot.Pose.y

        # Convert theta to quaternion for orientation
        quaternion = quaternion_from_euler(0, 0, self.robot.Pose.theta)  # roll and pitch are 0 for 2D case
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Log position
        position = odom_msg.pose.pose.position
        self.get_logger().info(f'Position - x: {position.x}, y: {position.y}, z: {position.z}')


class Robot:
    _PI = 3.14159265359
    _R_WHEEL = 0.094
    _D_WHEEL = 0.48

    def __init__(self):
        self.Pose = Pose(0, 0, 0)

    def updateOdometry(self, left_wheel_pos, right_wheel_pos):
        import math
        
        ds_left = self._R_WHEEL * left_wheel_pos
        ds_right = self._R_WHEEL * right_wheel_pos
        ds = (ds_left + ds_right) / 2
        dtheta = (ds_right - ds_left) / self._D_WHEEL
        self.Pose.x += ds * math.cos(self.Pose.theta)
        self.Pose.y += ds * math.sin(self.Pose.theta)
        self.Pose.theta += dtheta


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def main(args=None):
    rclpy.init(args=args)
    node = DiabloOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
