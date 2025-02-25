// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/JointState.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/joint_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "wearable_robot_interfaces/msg/detail/joint_state__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
cdr_serialize(
  const wearable_robot_interfaces::msg::JointState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: r_shoulder_angle
  cdr << ros_message.r_shoulder_angle;
  // Member: l_shoulder_angle
  cdr << ros_message.l_shoulder_angle;
  // Member: r_elbow_angle
  cdr << ros_message.r_elbow_angle;
  // Member: l_elbow_angle
  cdr << ros_message.l_elbow_angle;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wearable_robot_interfaces::msg::JointState & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: r_shoulder_angle
  cdr >> ros_message.r_shoulder_angle;

  // Member: l_shoulder_angle
  cdr >> ros_message.l_shoulder_angle;

  // Member: r_elbow_angle
  cdr >> ros_message.r_elbow_angle;

  // Member: l_elbow_angle
  cdr >> ros_message.l_elbow_angle;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
get_serialized_size(
  const wearable_robot_interfaces::msg::JointState & ros_message,
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
  // Member: r_shoulder_angle
  {
    size_t item_size = sizeof(ros_message.r_shoulder_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l_shoulder_angle
  {
    size_t item_size = sizeof(ros_message.l_shoulder_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r_elbow_angle
  {
    size_t item_size = sizeof(ros_message.r_elbow_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l_elbow_angle
  {
    size_t item_size = sizeof(ros_message.l_elbow_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wearable_robot_interfaces
max_serialized_size_JointState(
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

  // Member: r_shoulder_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: l_shoulder_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: r_elbow_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: l_elbow_angle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _JointState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const wearable_robot_interfaces::msg::JointState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _JointState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<wearable_robot_interfaces::msg::JointState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _JointState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const wearable_robot_interfaces::msg::JointState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _JointState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_JointState(full_bounded, 0);
}

static message_type_support_callbacks_t _JointState__callbacks = {
  "wearable_robot_interfaces::msg",
  "JointState",
  _JointState__cdr_serialize,
  _JointState__cdr_deserialize,
  _JointState__get_serialized_size,
  _JointState__max_serialized_size
};

static rosidl_message_type_support_t _JointState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_JointState__callbacks,
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
get_message_type_support_handle<wearable_robot_interfaces::msg::JointState>()
{
  return &wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::_JointState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wearable_robot_interfaces, msg, JointState)() {
  return &wearable_robot_interfaces::msg::typesupport_fastrtps_cpp::_JointState__handle;
}

#ifdef __cplusplus
}
#endif
