// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__FUNCTIONS_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wearable_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "wearable_robot_interfaces/msg/detail/imu_type__struct.h"

/// Initialize msg/IMUType message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wearable_robot_interfaces__msg__IMUType
 * )) before or use
 * wearable_robot_interfaces__msg__IMUType__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__init(wearable_robot_interfaces__msg__IMUType * msg);

/// Finalize msg/IMUType message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMUType__fini(wearable_robot_interfaces__msg__IMUType * msg);

/// Create msg/IMUType message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wearable_robot_interfaces__msg__IMUType__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__IMUType *
wearable_robot_interfaces__msg__IMUType__create();

/// Destroy msg/IMUType message.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMUType__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMUType__destroy(wearable_robot_interfaces__msg__IMUType * msg);

/// Check for msg/IMUType message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__are_equal(const wearable_robot_interfaces__msg__IMUType * lhs, const wearable_robot_interfaces__msg__IMUType * rhs);

/// Copy a msg/IMUType message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__copy(
  const wearable_robot_interfaces__msg__IMUType * input,
  wearable_robot_interfaces__msg__IMUType * output);

/// Initialize array of msg/IMUType messages.
/**
 * It allocates the memory for the number of elements and calls
 * wearable_robot_interfaces__msg__IMUType__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__Sequence__init(wearable_robot_interfaces__msg__IMUType__Sequence * array, size_t size);

/// Finalize array of msg/IMUType messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMUType__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMUType__Sequence__fini(wearable_robot_interfaces__msg__IMUType__Sequence * array);

/// Create array of msg/IMUType messages.
/**
 * It allocates the memory for the array and calls
 * wearable_robot_interfaces__msg__IMUType__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__IMUType__Sequence *
wearable_robot_interfaces__msg__IMUType__Sequence__create(size_t size);

/// Destroy array of msg/IMUType messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMUType__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMUType__Sequence__destroy(wearable_robot_interfaces__msg__IMUType__Sequence * array);

/// Check for msg/IMUType message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__Sequence__are_equal(const wearable_robot_interfaces__msg__IMUType__Sequence * lhs, const wearable_robot_interfaces__msg__IMUType__Sequence * rhs);

/// Copy an array of msg/IMUType messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMUType__Sequence__copy(
  const wearable_robot_interfaces__msg__IMUType__Sequence * input,
  wearable_robot_interfaces__msg__IMUType__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__FUNCTIONS_H_
