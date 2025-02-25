// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wearable_robot_interfaces:msg/DisplacementData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wearable_robot_interfaces/msg/detail/displacement_data__rosidl_typesupport_introspection_c.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wearable_robot_interfaces/msg/detail/displacement_data__functions.h"
#include "wearable_robot_interfaces/msg/detail/displacement_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `displacement`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wearable_robot_interfaces__msg__DisplacementData__init(message_memory);
}

void DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_fini_function(void * message_memory)
{
  wearable_robot_interfaces__msg__DisplacementData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__DisplacementData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "displacement",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__DisplacementData, displacement),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_members = {
  "wearable_robot_interfaces__msg",  // message namespace
  "DisplacementData",  // message name
  2,  // number of fields
  sizeof(wearable_robot_interfaces__msg__DisplacementData),
  DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_member_array,  // message members
  DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_init_function,  // function to initialize message memory (memory has to be allocated)
  DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_type_support_handle = {
  0,
  &DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, DisplacementData)() {
  DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_type_support_handle.typesupport_identifier) {
    DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DisplacementData__rosidl_typesupport_introspection_c__DisplacementData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
