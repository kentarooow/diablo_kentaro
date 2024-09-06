from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                '/home/kentaro/diablo_ws/src/diablo_kentaro/diablo_localization/config/ekf.yaml'
            ],
            remappings = [('odometry', 'diablo/odometry'),
                          ('imu/data', 'diablo/sensor/Imu'),]
        ),
    ])