from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('yapay_gps')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    fake_gps = Node(
        package='yapay_gps',
        executable='gps_gazebo.py',
        name='gps_gazebo',
        output='screen'
    )


    encoder = Node(
        package='yapay_gps',
        executable='encoder.py',
        name='encoder',
        output='screen',
        parameters=[{
            'in_topic': '/odom',
            'out_topic': '/wheel/odometry'
        }]
    )


    return LaunchDescription([ekf, fake_gps, encoder])
