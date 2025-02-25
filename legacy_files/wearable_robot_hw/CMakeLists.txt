cmake_minimum_required(VERSION 3.5)
project(wearable_robot_hw)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wearable_robot_interfaces REQUIRED)

# libserial 찾기
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(SERIAL REQUIRED libserial)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# USART_actuator_node
# add_executable(USART_actuator_node src/USART_actuator_node.cpp)
# ament_target_dependencies(USART_actuator_node
#   rclcpp
#   wearable_robot_interfaces
#   std_msgs
# )

# target_include_directories(USART_actuator_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${SERIAL_INCLUDE_DIRS}
# )

# target_link_libraries(USART_actuator_node
#   ${SERIAL_LIBRARIES}
# )


# CAN setting
add_executable(CAN_setting_node src/CAN_setting_node.cpp)
ament_target_dependencies(CAN_setting_node
  rclcpp
)


# CAN_data_extractor_node
add_executable(CAN_data_extractor_node src/CAN_data_extractor_node.cpp)
ament_target_dependencies(CAN_data_extractor_node
  rclcpp
  wearable_robot_interfaces
  std_msgs
)

target_include_directories(CAN_data_extractor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)

# CAN 명령 전송 노드 추가
add_executable(CAN_command_transmit_node src/CAN_command_transmit_node.cpp)
ament_target_dependencies(CAN_command_transmit_node
  rclcpp
  wearable_robot_interfaces
  std_msgs
)

target_include_directories(CAN_command_transmit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  CAN_data_extractor_node
  CAN_command_transmit_node
  CAN_setting_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
