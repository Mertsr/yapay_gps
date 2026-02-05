# yapay_gps (fake_gps)

ROS 2 tabanlı “sahte GPS” (NavSatFix) üretim paketi. Amaç, gerçek GNSS olmadan (yalnızca odometri + IMU ile) tutarlı bir NavSatFix akışı üretmek ve robot_localization ile GPS’siz bir yerelleştirme kurgusunda referans olarak kullanmaktır. Paket aynı zamanda tekerlek odometrisini sadeleştiren bir dönüştürücü ve seri port köprü scriptleri içerir.

- Paket adı: `yapay_gps`
- Diller: Python (ROS 2 ament_cmake + ament_cmake_python)
- Test edilen launch: `gazebo_nav.launch.py` (EKF + sahte GPS + encoder)




![WhatsApp Image 2026-02-06 at 01 39 57](https://github.com/user-attachments/assets/c4d6135a-289f-4bf3-b1d4-53781fbd320e)


## İçerik

```
fake_gps/
├─ CMakeLists.txt
├─ package.xml
├─ README.md
├─ config/
│  └─ ekf.yaml                 # robot_localization konfigürasyonu (2D, vx + yaw rate)
├─ launch/
│  ├─ dr_nav.launch.py         # EKF + fake_gps (varyant)
│  └─ gazebo_nav.launch.py     # EKF + gps_gazebo + encoder (önerilen)
└─ scripts/
   ├─ fake_gps.py              # Odom → NavSatFix (sabit origin + yaw hizası)
   ├─ fake_gps_real.py         # İlk odom’u spawn ofseti olarak kilitleyen varyant
   ├─ gps_gazebo.py            # Gazebo kullanımına uygun varyant (spawn ofseti kilitleme)
   ├─ encoder.py               # /odom → /wheel/odometry (sadece linear.x)
   ├─ encoder_real.py          # encoder.py ile aynı (birleştirilebilir)
   ├─ ros_ard.py               # Seri porttan IMU köprüsü (MPU9250 vb.)
   ├─ serial_ros2_bridge.py    # Seri porttan GPS/IMU/ENC köprüsü (WIP, notlara bakın)
   ├─ cmd_vel_subscriber.py    # /cmd_vel → Arduino komutları (WIP)
   ├─ arduino_command_subscriber.py # /cmd_vel → Arduino komutları (WIP)
   └─ lib/
      ├─ __init__.py
      └─ arduino_socket_client.py   # Çoklu Arduino soket istemcisi
```

## Nasıl çalışır?

- `gps_gazebo.py`/`fake_gps.py`, odometri (`/odom`) konumunu yerel ENU düzlemine hizalar (parametre `yaw0_deg`), seçtiğiniz bir WGS84 başlangıç konumunu (`origin_lat`, `origin_lon`, `origin_alt`) referans alarak küçük alan yaklaşımıyla (equirectangular) enlem-boylam üretir ve `sensor_msgs/NavSatFix` olarak (`/fix`) yayınlar.
- `encoder.py`, `/odom` içindeki hızları sadeleştirerek yalnızca `linear.x` taşıyan bir `nav_msgs/Odometry` mesajını `/wheel/odometry` olarak yayınlar. EKF’de tekerlek odometrisi girdisi olarak kullanılır.
- `config/ekf.yaml`, `robot_localization` paketi için 2D modda bir EKF yapılandırmasıdır:
  - Girişler: `/wheel/odometry` (yalnızca vx), `/imu/data` (yalnızca yaw rate)
  - GPS kasıtlı olarak füzyona katılmaz (GPS’siz yerelleştirme).

## Kurulum

Önkoşullar:
- ROS 2 (Humble/Jazzy veya uyumlu)
- Python 3
- Önerilen paketler:
  - `robot_localization`
  - `numpy` (encoder)
  - `pyserial` (yalnızca seri köprü scriptleri için)

Kurulum adımları:

```bash
# Çalışma alanını oluşturun
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Bu repoyu klonlayın
git clone https://github.com/Mertsr/fake_gps.git

# Bağımlılıkları kurun (robot_localization vb.)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Derleyin
colcon build --packages-select yapay_gps
source install/setup.bash
```

Not: `package.xml` içinde Python tabanlı bağımlılıkların bir kısmı henüz listelenmiyor olabilir. Sisteminizde eksik paket hatası alırsanız ilgili ROS mesaj paketlerini ve Python kütüphanelerini (örn. `python3-numpy`, `python3-serial`) elle kurun.

## Kullanım

Önerilen launch (Gazebo/Sim):

```bash
ros2 launch yapay_gps gazebo_nav.launch.py
```

Bu launch:
- `robot_localization` → `ekf_node` (config: `config/ekf.yaml`)
- `yapay_gps` → `gps_gazebo.py` (NavSatFix üretir, `/fix`)
- `yapay_gps` → `encoder.py` (tekerlek odometrisi üretir, `/wheel/odometry`)

Varsayılan parametreler:
- `gps_gazebo.py`: `origin_lat=2.0`, `origin_lon=1.5`, `origin_alt=0.0`, `yaw0_deg=0.0`, `odom_topic=/odom`, `navsat_topic=/fix`
- `encoder.py`: `in_topic=/odom`, `out_topic=/wheel/odometry`

Parametreleri değiştirme:
- Bu launch dosyası komut satırı argümanları tanımlamadığı için parametreleri çalışma anında set edebilirsiniz:
  ```bash
  # Node başlatıldıktan sonra:
  ros2 param set /gps_gazebo origin_lat 41.0082
  ros2 param set /gps_gazebo origin_lon 28.9784
  ros2 param set /gps_gazebo yaw0_deg 0.0
  ```
- Alternatif: `launch/gazebo_nav.launch.py` içinde Node `parameters=[{...}]` kısmını düzenleyin.

Beklenen topic’ler:
- Girdiler:
  - `/odom` (nav_msgs/Odometry) — GPS üretimi ve encoder girişi
  - `/imu/data` (sensor_msgs/Imu) — EKF yaw rate girişi (ör. `ros_ard.py` ile)
- Çıktılar:
  - `/fix` (sensor_msgs/NavSatFix) — Sahte GPS
  - `/wheel/odometry` (nav_msgs/Odometry) — Sadeleştirilmiş odometri (vx)

## Parametreler

Sahte GPS (gps_gazebo.py / fake_gps.py):
- `origin_lat` (float, deg): Başlangıç enlemi (WGS84)
- `origin_lon` (float, deg): Başlangıç boylamı (WGS84)
- `origin_alt` (float, m): Başlangıç irtifası
- `yaw0_deg` (float, deg): Odom eksenlerini ENU’ya hizalama açısı (x=Doğu, y=Kuzey için çoğunlukla 0)
- `odom_topic` (string): Odom kaynağı (örn. `/odom` veya EKF çıkışı `/odometry/filtered`)
- `navsat_topic` (string): NavSatFix çıkışı (varsayılan `/fix`)

Encoder (encoder.py):
- `in_topic` (string): Girdi odometri (varsayılan `/odom`)
- `out_topic` (string): Çıkış tekerlek odometrisi (varsayılan `/wheel/odometry`)

EKF (config/ekf.yaml):
- 2D mod (`two_d_mode: true`), `odom0` olarak `/wheel/odometry` (yalnız vx), `imu0` olarak `/imu/data` (yalnız yaw rate). `publish_tf: false`.

Seri köprü (opsiyonel):
- `ros_ard.py`: `port`, `baud`, `frame_id`, ölçek ve bias parametreleri
- `serial_ros2_bridge.py`: `port`, `baud`, `frame_gps`, `frame_imu`, `frame_enc`

## Doğrulama ve ipuçları

- Başlangıç kontrolü:
  - Robot odom=(0,0) iken `/fix` ≈ (`origin_lat`, `origin_lon`) olmalı.
- Eksen/yön hizası:
  - x yönünde +10 m → longitude artmalı (lat ~ sabit).
  - y yönünde +10 m → latitude artmalı (lon ~ sabit).
  - Tersi oluyorsa `yaw0_deg` ayarlayın.
- Ölçek kontrolü (küçük alan yaklaşımı):
  - Doğuya +10 m → Δlon ≈ 10 / (R·cos(lat)) · 180/π derece
  - Kuzeye +10 m → Δlat ≈ 10 / R · 180/π derece
- Frame:
  - `gps_gazebo.py` NavSatFix `header.frame_id = "map"` ayarlıyor. Entegrasyon yaptığınız düğümlerin beklentisine göre `"gps"` gibi bir frame kullanmak isteyebilirsiniz.
- EKF:
  - GPS şu an füzyona dahil edilmez; yalnızca IMU + tekerlek odometrisi ile çalışır. GPS füzyonu isterseniz `ekf.yaml`’ı genişletin.

## Bilinen durumlar ve sınırlamalar

- Küçük alan yaklaşımı: ENU→LLA dönüşümü küçük alanlar için uygundur. Çok geniş alanlarda/enlemlerde hata büyüyebilir. Gerekirse `geographiclib`/`pyproj` gibi projeksiyon kütüphaneleri entegre edilebilir.
- `serial_ros2_bridge.py`: Kodda `self.pub_fix` ve `self.pub_vel` yayıncıları tanımlanmadan kullanılıyor — GPS satırı geldiğinde hata üretebilir. Varsayılan launch’larda kullanılmaz; ihtiyaç halinde düzeltme gerektirir.
- `cmd_vel_subscriber.py` ve `arduino_command_subscriber.py`:
  - Shebang eksik ve `from libs.arduino_socket_client` import yolu repo’daki `scripts/lib/...` yapısıyla uyumsuz (paketleme yapılmadığı için doğrudan import edilemeyebilir). Bu scriptler WIP kabul edilmelidir.
- `encoder_real.py` = `encoder.py` ile aynı; tek dosyada toplanabilir.
- `package.xml`: Python tabanlı bağımlılıklar eksik olabilir (örn. `rclpy`, mesaj paketleri, `numpy`, `pyserial`). Sisteminizde yüklü değilse manuel kurulum gerekebilir.

## Yol haritası (öneriler)

- Launch argümanları ekleyip parametreleri CLI’dan geçilebilir hale getirmek.
- NavSatFix için gerçekçi gürültü/kovaryans ve drop-out simülasyonu.
- `serial_ros2_bridge.py`, `cmd_vel_subscriber.py` ve Arduino kütüphanesi için paketleme ve import düzenlemeleri.
- `package.xml` bağımlılıklarının tamamlanması, `encoder_real.py` sadeleştirmesi.

## Katkı ve lisans

- İyileştirme/PR’ler memnuniyetle karşılanır. Önemli değişiklikler için önce tartışma açın.
- Lisans: Apache License 2.0 (Apache-2.0)

## Hızlı başlatma özeti

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select yapay_gps
source install/setup.bash

# Launch (Gazebo/Sim)
ros2 launch yapay_gps gazebo_nav.launch.py

# Parametreleri çalışma anında güncelle (örnek)
ros2 param set /gps_gazebo origin_lat 41.0082
ros2 param set /gps_gazebo origin_lon 28.9784
ros2 param set /gps_gazebo yaw0_deg 0.0

# Doğrula
ros2 topic echo /fix
ros2 topic echo /wheel/odometry
```

---
Soruların veya katkı önerilerin için GitHub Issues açabilirsin.
