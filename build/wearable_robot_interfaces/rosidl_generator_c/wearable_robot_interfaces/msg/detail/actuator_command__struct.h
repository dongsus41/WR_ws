// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/ActuatorCommand.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_

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
// Member 'pwm'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/ActuatorCommand in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__ActuatorCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__uint8__Sequence pwm;
} wearable_robot_interfaces__msg__ActuatorCommand;

// Struct for a sequence of wearable_robot_interfaces__msg__ActuatorCommand.
typedef struct wearable_robot_interfaces__msg__ActuatorCommand__Sequence
{
  wearable_robot_interfaces__msg__ActuatorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__ActuatorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_
