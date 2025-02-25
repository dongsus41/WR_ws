cmake_minimum_required(VERSION 3.5)
project(wearable_robot_data_processing)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wearable_robot_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_socketcan_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)

# Include directories
include_directories(
  include
  ${ros2_socketcan_INCLUDE_DIRS}
)

# CAN Receiver Node
add_executable(can_data_processor src/can_data_processor.cpp)
ament_target_dependencies(can_data_processor
  rclcpp
  ros2_socketcan_msgs
  std_msgs
  wearable_robot_interfaces
)

# Parser Node
add_executable(data_parser_node
  src/data_parser_node.cpp
)
ament_target_dependencies(data_parser_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
)

# add_executable(displacement_data_logger src/displacement_data_logger_node.cpp)
# ament_target_dependencies(displacement_data_logger rclcpp wearable_robot_interfaces)

# Displacement Processing Node
add_executable(displacement_processing_node
  src/displacement_processing_node.cpp
)
ament_target_dependencies(displacement_processing_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
)

# IMU Processing Node
add_executable(IMU_processing_node
  src/IMU_processing_node.cpp
)
ament_target_dependencies(IMU_processing_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
)




# 실행 파일 설치
install(TARGETS
  can_data_processor
  data_parser_node
  #displacement_data_logger
  #displacement_processing_node
  IMU_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Include 파일 설치 (헤더 파일)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
