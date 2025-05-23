// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/TemperatureData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__STRUCT_H_

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
// Member 'temperature'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/TemperatureData in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__TemperatureData
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__float__Sequence temperature;
} wearable_robot_interfaces__msg__TemperatureData;

// Struct for a sequence of wearable_robot_interfaces__msg__TemperatureData.
typedef struct wearable_robot_interfaces__msg__TemperatureData__Sequence
{
  wearable_robot_interfaces__msg__TemperatureData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__TemperatureData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__STRUCT_H_
