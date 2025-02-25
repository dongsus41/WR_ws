// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/DisplacementRawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__STRUCT_H_

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
// Member 'displacement_raw'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/DisplacementRawData in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__DisplacementRawData
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__int32__Sequence displacement_raw;
} wearable_robot_interfaces__msg__DisplacementRawData;

// Struct for a sequence of wearable_robot_interfaces__msg__DisplacementRawData.
typedef struct wearable_robot_interfaces__msg__DisplacementRawData__Sequence
{
  wearable_robot_interfaces__msg__DisplacementRawData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__DisplacementRawData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__STRUCT_H_
