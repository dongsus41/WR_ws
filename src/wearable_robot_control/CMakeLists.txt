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

# 온도 제어 노드 추가
add_executable(actuator_temp_control_node
  src/actuator_control_node.cpp
)
ament_target_dependencies(actuator_temp_control_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
  ros2_socketcan_msgs
)

# 온도 로거 노드 추가
add_executable(temperature_logger_node
  src/temperature_logger_node.cpp
)
ament_target_dependencies(temperature_logger_node
  rclcpp
  std_msgs
  wearable_robot_interfaces
)

# 바디 컨트롤 노드는 주석 처리 (현재 파일명에 공백이 있어 문제 발생)
# add_executable(body_control_node
#   src/body_control_node\ copy.cpp
# )
# ament_target_dependencies(body_control_node
#   rclcpp
#   wearable_robot_interfaces
# )

# 허리 로봇 제어 노드 추가
# add_executable(waist_robot_control_node
#   src/waist_robot_control_node.cpp
# )
# ament_target_dependencies(waist_robot_control_node
#   rclcpp
#   wearable_robot_interfaces
# )

# 실행 파일 설치
install(TARGETS
  actuator_temp_control_node
  temperature_logger_node
  # body_control_node - 주석 처리
  #waist_robot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (있는 경우)
install(
  DIRECTORY include/
  DESTINATION include
  OPTIONAL
)

# 시작 스크립트 및 설정 파일 설치 (있는 경우)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
