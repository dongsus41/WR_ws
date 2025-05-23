// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wearable_robot_interfaces:msg/IMUData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wearable_robot_interfaces/msg/detail/imu_data__rosidl_typesupport_introspection_c.h"
#include "wearable_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wearable_robot_interfaces/msg/detail/imu_data__functions.h"
#include "wearable_robot_interfaces/msg/detail/imu_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `imu1`
// Member `imu2`
// Member `imu3`
// Member `imu4`
#include "wearable_robot_interfaces/msg/imu_type.h"
// Member `imu1`
// Member `imu2`
// Member `imu3`
// Member `imu4`
#include "wearable_robot_interfaces/msg/detail/imu_type__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void IMUData__rosidl_typesupport_introspection_c__IMUData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wearable_robot_interfaces__msg__IMUData__init(message_memory);
}

void IMUData__rosidl_typesupport_introspection_c__IMUData_fini_function(void * message_memory)
{
  wearable_robot_interfaces__msg__IMUData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUData, imu1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUData, imu2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUData, imu3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wearable_robot_interfaces__msg__IMUData, imu4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IMUData__rosidl_typesupport_introspection_c__IMUData_message_members = {
  "wearable_robot_interfaces__msg",  // message namespace
  "IMUData",  // message name
  5,  // number of fields
  sizeof(wearable_robot_interfaces__msg__IMUData),
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array,  // message members
  IMUData__rosidl_typesupport_introspection_c__IMUData_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUData__rosidl_typesupport_introspection_c__IMUData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle = {
  0,
  &IMUData__rosidl_typesupport_introspection_c__IMUData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wearable_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUData)() {
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUType)();
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUType)();
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUType)();
  IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wearable_robot_interfaces, msg, IMUType)();
  if (!IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle.typesupport_identifier) {
    IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
