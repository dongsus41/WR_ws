// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__struct.h"
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header
#include "wearable_robot_interfaces/msg/detail/imu_type__functions.h"  // imu1, imu2, imu3, imu4

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_wearable_robot_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_wearable_robot_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
size_t get_serialized_size_wearable_robot_interfaces__msg__IMUType(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_wearable_robot_interfaces__msg__IMUType(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType)();


using _IMURawData__ros_msg_type = wearable_robot_interfaces__msg__IMURawData;

static bool _IMURawData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IMURawData__ros_msg_type * ros_message = static_cast<const _IMURawData__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: can_id
  {
    cdr << ros_message->can_id;
  }

  // Field name: imu1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu1, cdr))
    {
      return false;
    }
  }

  // Field name: imu2
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu2, cdr))
    {
      return false;
    }
  }

  // Field name: imu3
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu3, cdr))
    {
      return false;
    }
  }

  // Field name: imu4
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu4, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _IMURawData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IMURawData__ros_msg_type * ros_message = static_cast<_IMURawData__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: can_id
  {
    cdr >> ros_message->can_id;
  }

  // Field name: imu1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu1))
    {
      return false;
    }
  }

  // Field name: imu2
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu2))
    {
      return false;
    }
  }

  // Field name: imu3
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu3))
    {
      return false;
    }
  }

  // Field name: imu4
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMUType
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu4))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wearable_robot_interfaces
size_t get_serialized_size_wearable_robot_interfaces__msg__IMURawData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IMURawData__ros_msg_type * ros_message = static_cast<const _IMURawData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name can_id
  {
    size_t item_size = sizeof(ros_message->can_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu1

  current_alignment += get_serialized_size_wearable_robot_interfaces__msg__IMUType(
    &(ros_message->imu1), current_alignment);
  // field.name imu2

  current_alignment += get_serialized_size_wearable_robot_interfaces__msg__IMUType(
    &(ros_message->imu2), current_alignment);
  // field.name imu3

  current_alignment += get_serialized_size_wearable_robot_interfaces__msg__IMUType(
    &(ros_message->imu3), current_alignment);
  // field.name imu4

  current_alignment += get_serialized_size_wearable_robot_interfaces__msg__IMUType(
    &(ros_message->imu4), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _IMURawData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_wearable_robot_interfaces__msg__IMURawData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wearable_robot_interfaces
size_t max_serialized_size_wearable_robot_interfaces__msg__IMURawData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: can_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: imu1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_wearable_robot_interfaces__msg__IMUType(
        full_bounded, current_alignment);
    }
  }
  // member: imu2
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_wearable_robot_interfaces__msg__IMUType(
        full_bounded, current_alignment);
    }
  }
  // member: imu3
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_wearable_robot_interfaces__msg__IMUType(
        full_bounded, current_alignment);
    }
  }
  // member: imu4
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_wearable_robot_interfaces__msg__IMUType(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _IMURawData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_wearable_robot_interfaces__msg__IMURawData(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IMURawData = {
  "wearable_robot_interfaces::msg",
  "IMURawData",
  _IMURawData__cdr_serialize,
  _IMURawData__cdr_deserialize,
  _IMURawData__get_serialized_size,
  _IMURawData__max_serialized_size
};

static rosidl_message_type_support_t _IMURawData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IMURawData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, wearable_robot_interfaces, msg, IMURawData)() {
  return &_IMURawData__type_support;
}

#if defined(__cplusplus)
}
#endif
