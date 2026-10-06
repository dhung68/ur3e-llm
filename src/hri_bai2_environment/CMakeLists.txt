cmake_minimum_required(VERSION 3.8)

project(hri_bai2_environment)

find_package(ament_cmake REQUIRED)

# Cài các file launch, world và cấu hình.
install(
  DIRECTORY launch worlds config srdf
  DESTINATION share/${PROJECT_NAME}
)

# Cài script để ROS có thể chạy nó.
install(
  PROGRAMS scripts/initialize_scene.py scripts/test_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
