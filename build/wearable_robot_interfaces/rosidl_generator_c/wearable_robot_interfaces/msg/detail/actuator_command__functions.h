// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wearable_robot_interfaces:msg/ActuatorCommand.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__FUNCTIONS_H_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wearable_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "wearable_robot_interfaces/msg/detail/actuator_command__struct.h"

/// Initialize msg/ActuatorCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wearable_robot_interfaces__msg__ActuatorCommand
 * )) before or use
 * wearable_robot_interfaces__msg__ActuatorCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__ActuatorCommand__init(wearable_robot_interfaces__msg__ActuatorCommand * msg);

/// Finalize msg/ActuatorCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__ActuatorCommand__fini(wearable_robot_interfaces__msg__ActuatorCommand * msg);

/// Create msg/ActuatorCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wearable_robot_interfaces__msg__ActuatorCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__ActuatorCommand *
wearable_robot_interfaces__msg__ActuatorCommand__create();

/// Destroy msg/ActuatorCommand message.
/**
 * It calls
 * wearable_robot_interfaces__msg__ActuatorCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__ActuatorCommand__destroy(wearable_robot_interfaces__msg__ActuatorCommand * msg);

/// Check for msg/ActuatorCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__ActuatorCommand__are_equal(const wearable_robot_interfaces__msg__ActuatorCommand * lhs, const wearable_robot_interfaces__msg__ActuatorCommand * rhs);

/// Copy a msg/ActuatorCommand message.
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
wearable_robot_interfaces__msg__ActuatorCommand__copy(
  const wearable_robot_interfaces__msg__ActuatorCommand * input,
  wearable_robot_interfaces__msg__ActuatorCommand * output);

/// Initialize array of msg/ActuatorCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * wearable_robot_interfaces__msg__ActuatorCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__init(wearable_robot_interfaces__msg__ActuatorCommand__Sequence * array, size_t size);

/// Finalize array of msg/ActuatorCommand messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__ActuatorCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__fini(wearable_robot_interfaces__msg__ActuatorCommand__Sequence * array);

/// Create array of msg/ActuatorCommand messages.
/**
 * It allocates the memory for the array and calls
 * wearable_robot_interfaces__msg__ActuatorCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
wearable_robot_interfaces__msg__ActuatorCommand__Sequence *
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__create(size_t size);

/// Destroy array of msg/ActuatorCommand messages.
/**
 * It calls
 * wearable_robot_interfaces__msg__ActuatorCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
void
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__destroy(wearable_robot_interfaces__msg__ActuatorCommand__Sequence * array);

/// Check for msg/ActuatorCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wearable_robot_interfaces
bool
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__are_equal(const wearable_robot_interfaces__msg__ActuatorCommand__Sequence * lhs, const wearable_robot_interfaces__msg__ActuatorCommand__Sequence * rhs);

/// Copy an array of msg/ActuatorCommand messages.
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
wearable_robot_interfaces__msg__ActuatorCommand__Sequence__copy(
  const wearable_robot_interfaces__msg__ActuatorCommand__Sequence * input,
  wearable_robot_interfaces__msg__ActuatorCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__FUNCTIONS_H_
