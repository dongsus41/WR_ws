// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_H_

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

// Struct defined in msg/JointState in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__JointState
{
  std_msgs__msg__Header header;
  double r_shoulder_angle;
  double l_shoulder_angle;
  double r_elbow_angle;
  double l_elbow_angle;
} wearable_robot_interfaces__msg__JointState;

// Struct for a sequence of wearable_robot_interfaces__msg__JointState.
typedef struct wearable_robot_interfaces__msg__JointState__Sequence
{
  wearable_robot_interfaces__msg__JointState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__JointState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_H_
