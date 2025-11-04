#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry

R_EARTH = 6378137.0  # WGS84 radius (m)

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps_sim')

        # Map(0,0,0) -> NavSatFix(0,0,0) olsun istiyorsan bunlar 0.0 kalsın
        self.declare_parameter('origin_lat', 2.0)
        self.declare_parameter('origin_lon', 1.5)
        self.declare_parameter('origin_alt', 0.0)

        self.declare_parameter('yaw0_deg', 0.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('navsat_topic', '/fix')

        # İlk odom’u spawn ofseti olarak kilitle
        self.spawn_locked = False
        self.x0 = self.y0 = self.z0 = 0.0

        self.lat0 = float(self.get_parameter('origin_lat').value)
        self.lon0 = float(self.get_parameter('origin_lon').value)
        self.alt0 = float(self.get_parameter('origin_alt').value)
        self.yaw0 = math.radians(float(self.get_parameter('yaw0_deg').value))
        odom_topic = self.get_parameter('odom_topic').value
        navsat_topic = self.get_parameter('navsat_topic').value

        self.fix_pub = self.create_publisher(NavSatFix, navsat_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)

        self.lat0_rad = math.radians(self.lat0)
        self.cos_lat0 = math.cos(self.lat0_rad)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if not self.spawn_locked:
            self.x0, self.y0, self.z0 = x, y, z
            self.spawn_locked = True

        x_rel = x - self.x0
        y_rel = y - self.y0
        z_rel = z - self.z0

        e =  x_rel*math.cos(self.yaw0) - y_rel*math.sin(self.yaw0)
        n =  x_rel*math.sin(self.yaw0) + y_rel*math.cos(self.yaw0)
        u =  z_rel

        dlat = (n / R_EARTH) * (180.0 / math.pi)
        dlon = (e / (R_EARTH * self.cos_lat0 if self.cos_lat0 != 0 else 1.0)) * (180.0 / math.pi)
        lat = self.lat0 + dlat
        lon = self.lon0 + dlon
        alt = self.alt0 + u

        fix = NavSatFix()
        fix.header = msg.header
        fix.header.frame_id = 'map'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat      # map (0,0,0) iken 0.0 yayınlar
        fix.longitude = lon
        fix.altitude = alt
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

def main():
    rclpy.init()
    node = FakeGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
