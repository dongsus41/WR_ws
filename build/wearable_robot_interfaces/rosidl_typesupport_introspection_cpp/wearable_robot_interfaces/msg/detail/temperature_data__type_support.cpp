// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/TemperatureData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wearable_robot_interfaces/msg/detail/temperature_data__struct.hpp"
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

void TemperatureData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wearable_robot_interfaces::msg::TemperatureData(_init);
}

void TemperatureData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wearable_robot_interfaces::msg::TemperatureData *>(message_memory);
  typed_message->~TemperatureData();
}

size_t size_function__TemperatureData__temperature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TemperatureData__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__TemperatureData__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__TemperatureData__temperature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TemperatureData_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::TemperatureData, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces::msg::TemperatureData, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__TemperatureData__temperature,  // size() function pointer
    get_const_function__TemperatureData__temperature,  // get_const(index) function pointer
    get_function__TemperatureData__temperature,  // get(index) function pointer
    resize_function__TemperatureData__temperature  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TemperatureData_message_members = {
  "wearable_robot_interfaces::msg",  // message namespace
  "TemperatureData",  // message name
  2,  // number of fields
  sizeof(wearable_robot_interfaces::msg::TemperatureData),
  TemperatureData_message_member_array,  // message members
  TemperatureData_init_function,  // function to initialize message memory (memory has to be allocated)
  TemperatureData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TemperatureData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TemperatureData_message_members,
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
get_message_type_support_handle<wearable_robot_interfaces::msg::TemperatureData>()
{
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::TemperatureData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wearable_robot_interfaces, msg, TemperatureData)() {
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::TemperatureData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
