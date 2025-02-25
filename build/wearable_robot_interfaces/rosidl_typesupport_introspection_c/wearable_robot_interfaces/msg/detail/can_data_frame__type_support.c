// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wearable_robot_interfaces:msg/CANDataFrame.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wearable_robot_interfaces/msg/detail/can_data_frame__rosidl_typesupport_introspection_c.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wearable_robot_interfaces/msg/detail/can_data_frame__functions.h"
#include "wearable_robot_interfaces/msg/detail/can_data_frame__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wearable_robot_interfaces__msg__CANDataFrame__init(message_memory);
}

void CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_fini_function(void * message_memory)
{
  wearable_robot_interfaces__msg__CANDataFrame__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__CANDataFrame, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    64,  // array size
    true,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__CANDataFrame, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_members = {
  "wearable_robot_interfaces__msg",  // message namespace
  "CANDataFrame",  // message name
  2,  // number of fields
  sizeof(wearable_robot_interfaces__msg__CANDataFrame),
  CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_member_array,  // message members
  CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_init_function,  // function to initialize message memory (memory has to be allocated)
  CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_type_support_handle = {
  0,
  &CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, CANDataFrame)() {
  CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_type_support_handle.typesupport_identifier) {
    CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CANDataFrame__rosidl_typesupport_introspection_c__CANDataFrame_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
