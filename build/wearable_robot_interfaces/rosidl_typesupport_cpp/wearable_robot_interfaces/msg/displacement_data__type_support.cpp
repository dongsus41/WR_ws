// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from wearable_robot_interfaces:msg/DisplacementData.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "wearable_robot_interfaces/msg/detail/displacement_data__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace wearable_robot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _DisplacementData_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _DisplacementData_type_support_ids_t;

static const _DisplacementData_type_support_ids_t _DisplacementData_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _DisplacementData_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _DisplacementData_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _DisplacementData_type_support_symbol_names_t _DisplacementData_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wearable_robot_interfaces, msg, DisplacementData)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wearable_robot_interfaces, msg, DisplacementData)),
  }
};

typedef struct _DisplacementData_type_support_data_t
{
  void * data[2];
} _DisplacementData_type_support_data_t;

static _DisplacementData_type_support_data_t _DisplacementData_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _DisplacementData_message_typesupport_map = {
  2,
  "wearable_robot_interfaces",
  &_DisplacementData_message_typesupport_ids.typesupport_identifier[0],
  &_DisplacementData_message_typesupport_symbol_names.symbol_name[0],
  &_DisplacementData_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t DisplacementData_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_DisplacementData_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace wearable_robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wearable_robot_interfaces::msg::DisplacementData>()
{
  return &::wearable_robot_interfaces::msg::rosidl_typesupport_cpp::DisplacementData_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, wearable_robot_interfaces, msg, DisplacementData)() {
  return get_message_type_support_handle<wearable_robot_interfaces::msg::DisplacementData>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
