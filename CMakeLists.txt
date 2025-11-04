cmake_minimum_required(VERSION 3.5)

project(yapay_gps)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install project launch files

install(
  DIRECTORY
    launch scripts config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/fake_gps.py scripts/encoder.py scripts/gps_gazebo.py scripts/fake_gps_real.py scripts/encoder_real.py scripts/serial_ros2_bridge.py scripts/ros_ard.py scripts/arduino_command_subscriber.py scripts/cmd_vel_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
