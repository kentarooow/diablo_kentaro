import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Ensure this is the correct import for publishing odometry
from motion_msgs.msg import LegMotors  # Adjust this to the correct import path
import socket
import struct
import math

class DiabloTransmission(Node):
    def __init__(self, target_ip, target_port):
        super().__init__('diablo_odometry')
        self.subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.listener_callback, 10)

        # Setup UDP socket for transmission
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def listener_callback(self, msg):
        # Transmit the leg motors data to the designated address
        self.transmit_data(msg)

        # Example of how you might handle the data locally or for other purposes:
        left_wheel_iq = msg.left_wheel_iq
        right_wheel_iq = msg.right_wheel_iq
        left_hip_iq = msg.left_hip_iq
        left_knee_iq = msg.left_knee_iq
        right_hip_iq = msg.right_hip_iq
        right_knee_iq = msg.right_knee_iq

        # Log the received data
        self.get_logger().info(
            f'Received - Left Hip IQ: {left_hip_iq}, Left Knee IQ: {left_knee_iq}, Left Wheel IQ: {left_wheel_iq}, '
            f'Right Hip IQ: {right_hip_iq}, Right Knee IQ: {right_knee_iq}, Right Wheel IQ: {right_wheel_iq}'
        )

    def transmit_data(self, msg):
        # Serialize the data to send it over UDP
        data = struct.pack(
            'ffffff',
            msg.left_hip_iq,
            msg.left_knee_iq,
            msg.left_wheel_iq,
            msg.right_hip_iq,
            msg.right_knee_iq,
            msg.right_wheel_iq
        )
        self.sock.sendto(data, (self.target_ip, self.target_port))
        self.get_logger().info(f'Sent data to {self.target_ip}:{self.target_port}')

def main(args=None):
    rclpy.init(args=args)

    # Set your target IP and port here
    target_ip = '127.0.0.1'  # Replace with your designated IP address
    target_port = 5000           # Replace with your designated port

    node = DiabloTransmission(target_ip, target_port)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
