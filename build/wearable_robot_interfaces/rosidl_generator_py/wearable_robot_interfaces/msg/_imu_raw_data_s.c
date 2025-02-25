// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__struct.h"
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool wearable_robot_interfaces__msg__imu_type__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * wearable_robot_interfaces__msg__imu_type__convert_to_py(void * raw_ros_message);
bool wearable_robot_interfaces__msg__imu_type__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * wearable_robot_interfaces__msg__imu_type__convert_to_py(void * raw_ros_message);
bool wearable_robot_interfaces__msg__imu_type__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * wearable_robot_interfaces__msg__imu_type__convert_to_py(void * raw_ros_message);
bool wearable_robot_interfaces__msg__imu_type__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * wearable_robot_interfaces__msg__imu_type__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool wearable_robot_interfaces__msg__imu_raw_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("wearable_robot_interfaces.msg._imu_raw_data.IMURawData", full_classname_dest, 54) == 0);
  }
  wearable_robot_interfaces__msg__IMURawData * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // can_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "can_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->can_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // imu1
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu1");
    if (!field) {
      return false;
    }
    if (!wearable_robot_interfaces__msg__imu_type__convert_from_py(field, &ros_message->imu1)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // imu2
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu2");
    if (!field) {
      return false;
    }
    if (!wearable_robot_interfaces__msg__imu_type__convert_from_py(field, &ros_message->imu2)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // imu3
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu3");
    if (!field) {
      return false;
    }
    if (!wearable_robot_interfaces__msg__imu_type__convert_from_py(field, &ros_message->imu3)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // imu4
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu4");
    if (!field) {
      return false;
    }
    if (!wearable_robot_interfaces__msg__imu_type__convert_from_py(field, &ros_message->imu4)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * wearable_robot_interfaces__msg__imu_raw_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IMURawData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("wearable_robot_interfaces.msg._imu_raw_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IMURawData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  wearable_robot_interfaces__msg__IMURawData * ros_message = (wearable_robot_interfaces__msg__IMURawData *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // can_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->can_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "can_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu1
    PyObject * field = NULL;
    field = wearable_robot_interfaces__msg__imu_type__convert_to_py(&ros_message->imu1);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu2
    PyObject * field = NULL;
    field = wearable_robot_interfaces__msg__imu_type__convert_to_py(&ros_message->imu2);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu3
    PyObject * field = NULL;
    field = wearable_robot_interfaces__msg__imu_type__convert_to_py(&ros_message->imu3);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu4
    PyObject * field = NULL;
    field = wearable_robot_interfaces__msg__imu_type__convert_to_py(&ros_message->imu4);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
