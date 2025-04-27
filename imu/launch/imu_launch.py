from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_imu_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
    ])

