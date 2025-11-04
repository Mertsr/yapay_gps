#!/usr/bin/env python3

import math, serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu

def _to_float(s):
    try:
        return float(s) if s != '' else None
    except Exception:
        return None

def euler_to_quat(roll, pitch, yaw):
    """roll/pitch/yaw radyan -> (x,y,z,w)"""
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
    x = sr*cp*cy - cr*sp*cy + cr*cp*sy + sr*sp*sy
    y = cr*sp*cy + sr*cp*cy + cr*cp*sy - sr*sp*sy
    z = cr*cp*sy - sr*sp*sy + sr*cp*cy - cr*sp*cy
    w = cr*cp*cy + sr*sp*sy
    return (x,y,z,w)

class SerialToROS(Node):
    def __init__(self):
        super().__init__('serial_to_ros')

        # Params
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_gps', 'gps')
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('frame_enc', 'base_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_gps = self.get_parameter('frame_gps').get_parameter_value().string_value
        self.frame_imu = self.get_parameter('frame_imu').get_parameter_value().string_value
        self.frame_enc = self.get_parameter('frame_enc').get_parameter_value().string_value

        self.pub_eul  = self.create_publisher(Vector3Stamped,'imu/euler', 10)
        self.pub_imu  = self.create_publisher(Imu,         'imu/data_raw', 10)
        self.pub_enc  = self.create_publisher(Vector3Stamped,'vel_odometry', 10)
        self.pub_move = self.create_publisher(Bool,        'encoder/is_moving', 10)

        # Serial
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0)
        except Exception as e:
            self.get_logger().fatal(f"Serial open failed on {port}@{baud}: {e}")
            raise

        # Timer: event-driven okuma
        self.timer = self.create_timer(0.0, self.spin_once)
        self.get_logger().info(f"Listening {port} @ {baud}")

    def now(self):
        return self.get_clock().now().to_msg()

    def spin_once(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line or line.startswith('#'):
            return
        parts = line.split(',')
        if len(parts) < 2:
            return

        tag = parts[1].upper()

        if tag == 'GPS' and len(parts) >= 7:
            lat = _to_float(parts[2]); lon = _to_float(parts[3])
            spd_kmh = _to_float(parts[4]); sats = _to_float(parts[5]); hdop = _to_float(parts[6])

            if lat is not None and lon is not None:
                fix = NavSatFix()
                fix.header.stamp = self.now()
                fix.header.frame_id = self.frame_gps
                fix.status.status  = NavSatStatus.STATUS_FIX if (sats or 0) >= 4 else NavSatStatus.STATUS_NO_FIX
                fix.status.service = NavSatStatus.SERVICE_GPS
                fix.latitude, fix.longitude = lat, lon
                fix.altitude = float('nan')
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.pub_fix.publish(fix)

                if spd_kmh is not None:
                    tw = TwistStamped()
                    tw.header.stamp = self.now()
                    tw.header.frame_id = self.frame_gps
                    tw.twist.linear.x = spd_kmh / 3.6  # m/s
                    self.pub_vel.publish(tw)

        elif tag == 'IMU' and len(parts) >= 5:
            roll_deg  = _to_float(parts[2]) or 0.0
            pitch_deg = _to_float(parts[3]) or 0.0
            yaw_deg   = _to_float(parts[4]) or 0.0

            r = math.radians(roll_deg)
            p = math.radians(pitch_deg)
            y = math.radians(yaw_deg)
            qx,qy,qz,qw = euler_to_quat(r,p,y)

            im = Imu()
            im.header.stamp = self.now()
            im.header.frame_id = self.frame_imu
            im.orientation.x, im.orientation.y, im.orientation.z, im.orientation.w = qx,qy,qz,qw
            im.orientation_covariance[0] = -1.0  # bilinmiyor
            self.pub_imu.publish(im)

            e = Vector3Stamped()
            e.header.stamp = self.now()
            e.header.frame_id = self.frame_imu
            e.vector.x, e.vector.y, e.vector.z = roll_deg, pitch_deg, yaw_deg
            self.pub_eul.publish(e)

        elif tag == 'ENC' and len(parts) >= 6:
            total    = _to_float(parts[2]) or 0.0
            movement = _to_float(parts[3]) or 0.0
            dist_m   = _to_float(parts[4]) or 0.0
            moving   = parts[5].strip() == '1'

            v = Vector3Stamped()
            v.header.stamp = self.now()
            v.header.frame_id = self.frame_enc
            v.vector.x, v.vector.y, v.vector.z = total, movement, dist_m
            self.pub_enc.publish(v)

            self.pub_move.publish(Bool(data=moving))

def main():
    rclpy.init()
    node = SerialToROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
