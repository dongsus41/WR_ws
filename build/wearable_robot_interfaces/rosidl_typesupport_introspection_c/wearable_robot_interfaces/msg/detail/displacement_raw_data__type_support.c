// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wearable_robot_interfaces:msg/DisplacementRawData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__rosidl_typesupport_introspection_c.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__functions.h"
#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `displacement_raw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wearable_robot_interfaces__msg__DisplacementRawData__init(message_memory);
}

void DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_fini_function(void * message_memory)
{
  wearable_robot_interfaces__msg__DisplacementRawData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__DisplacementRawData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "displacement_raw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__DisplacementRawData, displacement_raw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_members = {
  "wearable_robot_interfaces__msg",  // message namespace
  "DisplacementRawData",  // message name
  2,  // number of fields
  sizeof(wearable_robot_interfaces__msg__DisplacementRawData),
  DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_member_array,  // message members
  DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_init_function,  // function to initialize message memory (memory has to be allocated)
  DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_type_support_handle = {
  0,
  &DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, DisplacementRawData)() {
  DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_type_support_handle.typesupport_identifier) {
    DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DisplacementRawData__rosidl_typesupport_introspection_c__DisplacementRawData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
