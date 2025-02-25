// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wearable_robot_interfaces/msg/detail/imu_type__rosidl_typesupport_introspection_c.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wearable_robot_interfaces/msg/detail/imu_type__functions.h"
#include "wearable_robot_interfaces/msg/detail/imu_type__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void IMUType__rosidl_typesupport_introspection_c__IMUType_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wearable_robot_interfaces__msg__IMUType__init(message_memory);
}

void IMUType__rosidl_typesupport_introspection_c__IMUType_fini_function(void * message_memory)
{
  wearable_robot_interfaces__msg__IMUType__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IMUType__rosidl_typesupport_introspection_c__IMUType_message_member_array[3] = {
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUType, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUType, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUType, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IMUType__rosidl_typesupport_introspection_c__IMUType_message_members = {
  "wearable_robot_interfaces__msg",  // message namespace
  "IMUType",  // message name
  3,  // number of fields
  sizeof(wearable_robot_interfaces__msg__IMUType),
  IMUType__rosidl_typesupport_introspection_c__IMUType_message_member_array,  // message members
  IMUType__rosidl_typesupport_introspection_c__IMUType_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUType__rosidl_typesupport_introspection_c__IMUType_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IMUType__rosidl_typesupport_introspection_c__IMUType_message_type_support_handle = {
  0,
  &IMUType__rosidl_typesupport_introspection_c__IMUType_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUType)() {
  if (!IMUType__rosidl_typesupport_introspection_c__IMUType_message_type_support_handle.typesupport_identifier) {
    IMUType__rosidl_typesupport_introspection_c__IMUType_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IMUType__rosidl_typesupport_introspection_c__IMUType_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
