// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wearable_robot_interfaces/msg/detail/imu_type__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wearable_robot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void IMUType_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wearable_robot_interfaces::msg::IMUType(_init);
}

void IMUType_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wearable_robot_interfaces::msg::IMUType *>(message_memory);
  typed_message->~IMUType();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IMUType_message_member_array[3] = {
  {
    "roll",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::IMUType, roll),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pitch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::IMUType, pitch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::IMUType, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IMUType_message_members = {
  "wearable_robot_interfaces::msg",  // message namespace
  "IMUType",  // message name
  3,  // number of fields
  sizeof(wearable_robot_interfaces::msg::IMUType),
  IMUType_message_member_array,  // message members
  IMUType_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUType_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IMUType_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IMUType_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace wearable_robot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wearable_robot_interfaces::msg::IMUType>()
{
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::IMUType_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wearable_robot_interfaces, msg, IMUType)() {
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::IMUType_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
