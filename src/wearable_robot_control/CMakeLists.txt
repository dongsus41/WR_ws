cmake_minimum_required(VERSION 3.5)
project(wearable_robot_control)

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
find_package(ros2_socketcan_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(std_srvs REQUIRED)

# 온도 제어 노드
add_executable(actuator_control_node
  src/actuator_control_node.cpp
)
ament_target_dependencies(actuator_control_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
  ros2_socketcan_msgs
)

# 명령 노드
add_executable(command_send_node
  src/command_send_node.cpp
)
ament_target_dependencies(command_send_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
  ros2_socketcan_msgs
)


# 실행 파일 설치
install(TARGETS
  actuator_control_node
  command_send_node
  DESTINATION lib/${PROJECT_NAME}
)

# 설정 파일 및 런치 파일 설치
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 헤더 파일 설치 (있는 경우)
install(
  DIRECTORY include/
  DESTINATION include
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
