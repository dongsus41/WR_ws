// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/DisplacementRawData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__struct.hpp"
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

void DisplacementRawData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wearable_robot_interfaces::msg::DisplacementRawData(_init);
}

void DisplacementRawData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wearable_robot_interfaces::msg::DisplacementRawData *>(message_memory);
  typed_message->~DisplacementRawData();
}

size_t size_function__DisplacementRawData__displacement_raw(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DisplacementRawData__displacement_raw(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__DisplacementRawData__displacement_raw(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void resize_function__DisplacementRawData__displacement_raw(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DisplacementRawData_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::DisplacementRawData, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "displacement_raw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::DisplacementRawData, displacement_raw),  // bytes offset in struct
    nullptr,  // default value
    size_function__DisplacementRawData__displacement_raw,  // size() function pointer
    get_const_function__DisplacementRawData__displacement_raw,  // get_const(index) function pointer
    get_function__DisplacementRawData__displacement_raw,  // get(index) function pointer
    resize_function__DisplacementRawData__displacement_raw  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DisplacementRawData_message_members = {
  "wearable_robot_interfaces::msg",  // message namespace
  "DisplacementRawData",  // message name
  2,  // number of fields
  sizeof(wearable_robot_interfaces::msg::DisplacementRawData),
  DisplacementRawData_message_member_array,  // message members
  DisplacementRawData_init_function,  // function to initialize message memory (memory has to be allocated)
  DisplacementRawData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DisplacementRawData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DisplacementRawData_message_members,
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
get_message_type_support_handle<wearable_robot_interfaces::msg::DisplacementRawData>()
{
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::DisplacementRawData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wearable_robot_interfaces, msg, DisplacementRawData)() {
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::DisplacementRawData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
