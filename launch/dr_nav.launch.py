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
        executable='fake_gps',
        name='fake_gps',
        output='screen',
        parameters=[{
            # Başlangıç WGS84 referansı
            'origin_lat': 41.0082,       # örn. İstanbul Kadıköy
            # WGS84 referans koordinatları
            'origin_lon': 28.9784,
            'origin_alt': 0.0,          # metre
            # Odom eksenlerinin ENU’ya hizası
            # odom-x doğu, odom-y kuzey olacak şekilde ofset ver
            'yaw0_deg': 0.0,             # odom x ekseni doğuya bakıyorsa 0
            # Giriş/çıkış
            'odom_topic': '/odom',
            'navsat_topic': '/fix'
        }]
    )

    return LaunchDescription([ekf, fake_gps])

