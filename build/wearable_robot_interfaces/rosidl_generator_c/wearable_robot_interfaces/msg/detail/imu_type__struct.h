// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__STRUCT_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/IMUType in the package wearable_robot_interfaces.
typedef struct wearable_robot_interfaces__msg__IMUType
{
  int32_t roll;
  int32_t pitch;
  int32_t yaw;
} wearable_robot_interfaces__msg__IMUType;

// Struct for a sequence of wearable_robot_interfaces__msg__IMUType.
typedef struct wearable_robot_interfaces__msg__IMUType__Sequence
{
  wearable_robot_interfaces__msg__IMUType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wearable_robot_interfaces__msg__IMUType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__STRUCT_H_
