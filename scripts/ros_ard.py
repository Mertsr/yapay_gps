#!/usr/bin/env python3
from array import array
import re, math, serial, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header

NUM_RE = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')

def deg2rad(x): return x * math.pi / 180.0
def g_to_ms2(x): return x * 9.80665

class Mpu9250Bridge(Node):
    def __init__(self):
        super().__init__('mpu9250_bridge')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        # raw=True -> int16 counts gelir, burada ölçeklenir.
        # raw=False -> Arduino SI birimlerinde yollar (accel: m/s^2, gyro: deg/s, mag: uT)
        self.declare_parameter('raw', True)
        # MPU9250 varsayılan ölçekler (Arduino'da farklıysa değiştir)
        self.declare_parameter('accel_lsb_per_g', 16384.0)     # ±2g
        self.declare_parameter('gyro_lsb_per_dps', 131.0)      # ±250 dps
        self.declare_parameter('mag_uT_per_lsb', 0.15)         # AK8963 16-bit approx (ASA uygulanmıyor)
        # Basit bias düzeltmeleri (kalibrasyonla güncelle)
        self.declare_parameter('accel_bias', [0.0, 0.0, 0.0])  # m/s^2
        self.declare_parameter('gyro_bias',  [0.0, 0.0, 0.0])  # rad/s
        self.declare_parameter('mag_bias',   [0.0, 0.0, 0.0])  # uT
        self.declare_parameter('gravity_comp', False)

        # Param değerleri
        port  = self.get_parameter('port').value
        baud  = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.raw = bool(self.get_parameter('raw').value)
        self.accel_lsb_per_g = float(self.get_parameter('accel_lsb_per_g').value)
        self.gyro_lsb_per_dps = float(self.get_parameter('gyro_lsb_per_dps').value)
        self.mag_uT_per_lsb = float(self.get_parameter('mag_uT_per_lsb').value)
        self.accel_bias = [float(x) for x in self.get_parameter('accel_bias').value]
        self.gyro_bias  = [float(x) for x in self.get_parameter('gyro_bias').value]
        self.mag_bias   = [float(x) for x in self.get_parameter('mag_bias').value]
        self.gravity_comp = bool(self.get_parameter('gravity_comp').value)

        # QoS: RELIABLE (abone tarafı RELIABLE istiyorsa uyumlu olur)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', qos)
        self.pub_accel_raw = self.create_publisher(Vector3Stamped, '/imu/accel_raw', qos)
        self.pub_gyro_raw  = self.create_publisher(Vector3Stamped, '/imu/gyro_raw', qos)

        # Kovaryanslar (9 elemanlı double dizi)
        self.linacc_cov = array('d', [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ])
        self.angvel_cov = array('d', [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ])
        self.orient_cov = array('d', [
            -1.0, 0.0, 0.0,
             0.0,-1.0, 0.0,
             0.0, 0.0,-1.0
        ])  # orientation yayınlamıyoruz -> -1

        # Seri port
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
            time.sleep(0.2)
            self.get_logger().info(f"Serial open: {port} @ {baud}")
        except Exception as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise

        self.timer = self.create_timer(0.01, self.read_once)  # ~100 Hz

    def parse_line(self, line: str):
        """
        Satırdan 9 sayı ayıklar (ax,ay,az,gx,gy,gz,mx,my,mz).
        CSV:   '100,-20,16300,0,0,0,120,30,-55'
        Metin: 'ACCEL: 100, -20, 16300 | GYRO: 0, 0, 0 | MAG: 120, 30, -55'
        """
        nums = NUM_RE.findall(line)
        if len(nums) < 9:
            return None
        vals = [float(n) for n in nums[:9]]
        return tuple(vals)

    def scale_units(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """
        raw=True: counts -> (m/s^2, rad/s, uT)
        raw=False: accel m/s^2, gyro deg/s, mag uT; gyro rad/s'a çevrilir
        """
        if self.raw:
            ax = g_to_ms2(ax / self.accel_lsb_per_g)
            ay = g_to_ms2(ay / self.accel_lsb_per_g)
            az = g_to_ms2(az / self.accel_lsb_per_g)
            gx = deg2rad(gx / self.gyro_lsb_per_dps)
            gy = deg2rad(gy / self.gyro_lsb_per_dps)
            gz = deg2rad(gz / self.gyro_lsb_per_dps)
            mx = mx * self.mag_uT_per_lsb
            my = my * self.mag_uT_per_lsb
            mz = mz * self.mag_uT_per_lsb
        else:
            gx = deg2rad(gx); gy = deg2rad(gy); gz = deg2rad(gz)

        # bias düzelt
        ax -= self.accel_bias[0]; ay -= self.accel_bias[1]; az -= self.accel_bias[2]
        gx -= self.gyro_bias[0];  gy -= self.gyro_bias[1];  gz -= self.gyro_bias[2]
        mx -= self.mag_bias[0];   my -= self.mag_bias[1];   mz -= self.mag_bias[2]

        if self.gravity_comp:
            az -= 9.80665  # 1g telafisi

        return ax, ay, az, gx, gy, gz, mx, my, mz

    def read_once(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            parsed = self.parse_line(line)
            if not parsed:
                return
            ax, ay, az, gx, gy, gz, mx, my, mz = self.scale_units(*parsed)

            now = self.get_clock().now().to_msg()
            hdr = Header(stamp=now, frame_id=self.frame_id)

            # IMU
            msg = Imu()
            msg.header = hdr
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.orientation_covariance         = self.orient_cov
            msg.angular_velocity_covariance    = self.angvel_cov
            msg.linear_acceleration_covariance = self.linacc_cov
            self.pub_imu.publish(msg)

            # MAG (uT -> Tesla)
            m = MagneticField()
            m.header = hdr
            m.magnetic_field.x = mx * 1e-6
            m.magnetic_field.y = my * 1e-6
            m.magnetic_field.z = mz * 1e-6
            self.pub_mag.publish(m)

            # Debug vektörleri
            a_raw = Vector3Stamped(); a_raw.header = hdr
            a_raw.vector.x, a_raw.vector.y, a_raw.vector.z = ax, ay, az
            g_raw = Vector3Stamped(); g_raw.header = hdr
            g_raw.vector.x, g_raw.vector.y, g_raw.vector.z = gx, gy, gz
            self.pub_accel_raw.publish(a_raw)
            self.pub_gyro_raw.publish(g_raw)

        except Exception as e:
            self.get_logger().warn(f"Read/parsing error: {e}")

def main():
    rclpy.init()
    node = Mpu9250Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
