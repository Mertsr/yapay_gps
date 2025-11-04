#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry

R_EARTH = 6378137.0  # WGS84 yarıçapı (m)

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('yaw0_deg', 0.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('navsat_topic', '/fix')

        self.lat0 = float(self.get_parameter('origin_lat').value)
        self.lon0 = float(self.get_parameter('origin_lon').value)
        self.alt0 = float(self.get_parameter('origin_alt').value)
        self.yaw0 = math.radians(float(self.get_parameter('yaw0_deg').value))

        odom_topic = self.get_parameter('odom_topic').value
        navsat_topic = self.get_parameter('navsat_topic').value

        self.fix_pub = self.create_publisher(NavSatFix, navsat_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)

        # Precompute
        self.lat0_rad = math.radians(self.lat0)
        self.cos_lat0 = math.cos(self.lat0_rad)

    def odom_cb(self, msg: Odometry):
        # Odom’da x,y (m) → yerel DR koordinatı
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Odom eksenlerini ENU’ya çevir (yaw0 ile hizalama)
        # ENU: east = e, north = n
        e =  x*math.cos(self.yaw0) - y*math.sin(self.yaw0)
        n =  x*math.sin(self.yaw0) + y*math.cos(self.yaw0)
        u =  z  # iki boyut modunda genelde 0

        # ENU → WGS84 (küçük alan yaklaşımı)
        dlat = (n / R_EARTH) * (180.0 / math.pi)
        dlon = (e / (R_EARTH * self.cos_lat0)) * (180.0 / math.pi)
        lat = self.lat0 + dlat
        lon = self.lon0 + dlon
        alt = self.alt0 + u

        fix = NavSatFix()
        fix.header = msg.header
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt

        # Basit bir kovaryans (DR olduğumuzu belirtmek için nispeten büyük)
        # diag: [lat_var, lon_var, alt_var] (deg^2, deg^2, m^2 değil! NavSatFix'te birimleri metre^2 beklenir)
        # Dönüşümle uğraşmamak için “unknown” kullanmak istersen aşağıdaki gibi yap:
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

