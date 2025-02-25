// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/CANData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__STRUCT_H_

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

// Struct defined in msg/CANData in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__CANData
{
  std_msgs__msg__Header header;
  uint32_t can_id;
  int32_t data[24];
} wearable_robot_interfaces__msg__CANData;

// Struct for a sequence of wearable_robot_interfaces__msg__CANData.
typedef struct wearable_robot_interfaces__msg__CANData__Sequence
{
  wearable_robot_interfaces__msg__CANData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__CANData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__STRUCT_H_
