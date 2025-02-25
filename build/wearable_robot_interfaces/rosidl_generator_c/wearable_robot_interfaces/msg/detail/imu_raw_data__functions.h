// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__FUNCTIONS_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wearable_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "wearable_robot_interfaces/msg/detail/imu_raw_data__struct.h"

/// Initialize msg/IMURawData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wearable_robot_interfaces__msg__IMURawData
 * )) before or use
 * wearable_robot_interfaces__msg__IMURawData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMURawData__init(wearable_robot_interfaces__msg__IMURawData * msg);

/// Finalize msg/IMURawData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMURawData__fini(wearable_robot_interfaces__msg__IMURawData * msg);

/// Create msg/IMURawData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wearable_robot_interfaces__msg__IMURawData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__IMURawData *
wearable_robot_interfaces__msg__IMURawData__create();

/// Destroy msg/IMURawData message.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMURawData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMURawData__destroy(wearable_robot_interfaces__msg__IMURawData * msg);

/// Check for msg/IMURawData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMURawData__are_equal(const wearable_robot_interfaces__msg__IMURawData * lhs, const wearable_robot_interfaces__msg__IMURawData * rhs);

/// Copy a msg/IMURawData message.
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
wearable_robot_interfaces__msg__IMURawData__copy(
  const wearable_robot_interfaces__msg__IMURawData * input,
  wearable_robot_interfaces__msg__IMURawData * output);

/// Initialize array of msg/IMURawData messages.
/**
 * It allocates the memory for the number of elements and calls
 * wearable_robot_interfaces__msg__IMURawData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMURawData__Sequence__init(wearable_robot_interfaces__msg__IMURawData__Sequence * array, size_t size);

/// Finalize array of msg/IMURawData messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMURawData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMURawData__Sequence__fini(wearable_robot_interfaces__msg__IMURawData__Sequence * array);

/// Create array of msg/IMURawData messages.
/**
 * It allocates the memory for the array and calls
 * wearable_robot_interfaces__msg__IMURawData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__IMURawData__Sequence *
wearable_robot_interfaces__msg__IMURawData__Sequence__create(size_t size);

/// Destroy array of msg/IMURawData messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__IMURawData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__IMURawData__Sequence__destroy(wearable_robot_interfaces__msg__IMURawData__Sequence * array);

/// Check for msg/IMURawData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__IMURawData__Sequence__are_equal(const wearable_robot_interfaces__msg__IMURawData__Sequence * lhs, const wearable_robot_interfaces__msg__IMURawData__Sequence * rhs);

/// Copy an array of msg/IMURawData messages.
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
wearable_robot_interfaces__msg__IMURawData__Sequence__copy(
  const wearable_robot_interfaces__msg__IMURawData__Sequence * input,
  wearable_robot_interfaces__msg__IMURawData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__FUNCTIONS_H_
