// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'imu1'
// Member 'imu2'
// Member 'imu3'
// Member 'imu4'
#include "wearable_robot_interfaces/msg/detail/imu_type__struct.h"

// Struct defined in msg/IMURawData in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__IMURawData
{
  std_msgs__msg__Header header;
  uint32_t can_id;
  wearable_robot_interfaces__msg__IMUType imu1;
  wearable_robot_interfaces__msg__IMUType imu2;
  wearable_robot_interfaces__msg__IMUType imu3;
  wearable_robot_interfaces__msg__IMUType imu4;
} wearable_robot_interfaces__msg__IMURawData;

// Struct for a sequence of wearable_robot_interfaces__msg__IMURawData.
typedef struct wearable_robot_interfaces__msg__IMURawData__Sequence
{
  wearable_robot_interfaces__msg__IMURawData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__IMURawData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_H_
