// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/IMUData.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/imu_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "wearable_robot_interfaces/msg/detail/imu_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace wearable_robot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const wearable_robot_interfaces::msg::IMUType &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  wearable_robot_interfaces::msg::IMUType &);
size_t get_serialized_size(
  const wearable_robot_interfaces::msg::IMUType &,
  size_t current_alignment);
size_t
max_serialized_size_IMUType(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace wearable_robot_interfaces

namespace wearable_robot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const wearable_robot_interfaces::msg::IMUType &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  wearable_robot_interfaces::msg::IMUType &);
size_t get_serialized_size(
  const wearable_robot_interfaces::msg::IMUType &,
  size_t current_alignment);
size_t
max_serialized_size_IMUType(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace wearable_robot_interfaces

namespace wearable_robot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const wearable_robot_interfaces::msg::IMUType &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  wearable_robot_interfaces::msg::IMUType &);
size_t get_serialized_size(
  const wearable_robot_interfaces::msg::IMUType &,
  size_t current_alignment);
size_t
max_serialized_size_IMUType(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace wearable_robot_interfaces

namespace wearable_robot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const wearable_robot_interfaces::msg::IMUType &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  wearable_robot_interfaces::msg::IMUType &);
size_t get_serialized_size(
  const wearable_robot_interfaces::msg::IMUType &,
  size_t current_alignment);
size_t
max_serialized_size_IMUType(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace wearable_robot_interfaces


namespace wearable_robot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
cdr_serialize(
  const wearable_robot_interfaces::msg::IMUData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: imu1
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.imu1,
    cdr);
  // Member: imu2
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.imu2,
    cdr);
  // Member: imu3
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.imu3,
    cdr);
  // Member: imu4
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.imu4,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wearable_robot_interfaces::msg::IMUData & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: imu1
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.imu1);

  // Member: imu2
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.imu2);

  // Member: imu3
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.imu3);

  // Member: imu4
  wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.imu4);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
get_serialized_size(
  const wearable_robot_interfaces::msg::IMUData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: imu1

  current_alignment +=
    wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.imu1, current_alignment);
  // Member: imu2

  current_alignment +=
    wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.imu2, current_alignment);
  // Member: imu3

  current_alignment +=
    wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.imu3, current_alignment);
  // Member: imu4

  current_alignment +=
    wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.imu4, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
max_serialized_size_IMUData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: imu1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_IMUType(
        full_bounded, current_alignment);
    }
  }

  // Member: imu2
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_IMUType(
        full_bounded, current_alignment);
    }
  }

  // Member: imu3
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_IMUType(
        full_bounded, current_alignment);
    }
  }

  // Member: imu4
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_IMUType(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _IMUData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const wearable_robot_interfaces::msg::IMUData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IMUData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<wearable_robot_interfaces::msg::IMUData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IMUData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const wearable_robot_interfaces::msg::IMUData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IMUData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_IMUData(full_bounded, 0);
}

static message_type_support_callbacks_t _IMUData__callbacks = {
  "wearable_robot_interfaces::msg",
  "IMUData",
  _IMUData__cdr_serialize,
  _IMUData__cdr_deserialize,
  _IMUData__get_serialized_size,
  _IMUData__max_serialized_size
};

static rosidl_message_type_support_t _IMUData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IMUData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace wearable_robot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<wearable_robot_interfaces::msg::IMUData>()
{
  return &wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::_IMUData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wearable_robot_interfaces, msg, IMUData)() {
  return &wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::_IMUData__handle;
}

#ifdef __cplusplus
}
#endif
