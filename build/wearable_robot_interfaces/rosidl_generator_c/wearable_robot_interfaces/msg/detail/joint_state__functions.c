// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wearable_robot_interfaces:msg/JointState.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/joint_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
wearable_robot_interfaces__msg__JointState__init(wearable_robot_interfaces__msg__JointState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wearable_robot_interfaces__msg__JointState__fini(msg);
    return false;
  }
  // r_shoulder_angle
  // l_shoulder_angle
  // r_elbow_angle
  // l_elbow_angle
  return true;
}

void
wearable_robot_interfaces__msg__JointState__fini(wearable_robot_interfaces__msg__JointState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // r_shoulder_angle
  // l_shoulder_angle
  // r_elbow_angle
  // l_elbow_angle
}

bool
wearable_robot_interfaces__msg__JointState__are_equal(const wearable_robot_interfaces__msg__JointState * lhs, const wearable_robot_interfaces__msg__JointState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // r_shoulder_angle
  if (lhs->r_shoulder_angle != rhs->r_shoulder_angle) {
    return false;
  }
  // l_shoulder_angle
  if (lhs->l_shoulder_angle != rhs->l_shoulder_angle) {
    return false;
  }
  // r_elbow_angle
  if (lhs->r_elbow_angle != rhs->r_elbow_angle) {
    return false;
  }
  // l_elbow_angle
  if (lhs->l_elbow_angle != rhs->l_elbow_angle) {
    return false;
  }
  return true;
}

bool
wearable_robot_interfaces__msg__JointState__copy(
  const wearable_robot_interfaces__msg__JointState * input,
  wearable_robot_interfaces__msg__JointState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // r_shoulder_angle
  output->r_shoulder_angle = input->r_shoulder_angle;
  // l_shoulder_angle
  output->l_shoulder_angle = input->l_shoulder_angle;
  // r_elbow_angle
  output->r_elbow_angle = input->r_elbow_angle;
  // l_elbow_angle
  output->l_elbow_angle = input->l_elbow_angle;
  return true;
}

wearable_robot_interfaces__msg__JointState *
wearable_robot_interfaces__msg__JointState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__JointState * msg = (wearable_robot_interfaces__msg__JointState *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__JointState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wearable_robot_interfaces__msg__JointState));
  bool success = wearable_robot_interfaces__msg__JointState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wearable_robot_interfaces__msg__JointState__destroy(wearable_robot_interfaces__msg__JointState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wearable_robot_interfaces__msg__JointState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wearable_robot_interfaces__msg__JointState__Sequence__init(wearable_robot_interfaces__msg__JointState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__JointState * data = NULL;

  if (size) {
    data = (wearable_robot_interfaces__msg__JointState *)allocator.zero_allocate(size, sizeof(wearable_robot_interfaces__msg__JointState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wearable_robot_interfaces__msg__JointState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wearable_robot_interfaces__msg__JointState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
wearable_robot_interfaces__msg__JointState__Sequence__fini(wearable_robot_interfaces__msg__JointState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      wearable_robot_interfaces__msg__JointState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

wearable_robot_interfaces__msg__JointState__Sequence *
wearable_robot_interfaces__msg__JointState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__JointState__Sequence * array = (wearable_robot_interfaces__msg__JointState__Sequence *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__JointState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wearable_robot_interfaces__msg__JointState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wearable_robot_interfaces__msg__JointState__Sequence__destroy(wearable_robot_interfaces__msg__JointState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wearable_robot_interfaces__msg__JointState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wearable_robot_interfaces__msg__JointState__Sequence__are_equal(const wearable_robot_interfaces__msg__JointState__Sequence * lhs, const wearable_robot_interfaces__msg__JointState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wearable_robot_interfaces__msg__JointState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wearable_robot_interfaces__msg__JointState__Sequence__copy(
  const wearable_robot_interfaces__msg__JointState__Sequence * input,
  wearable_robot_interfaces__msg__JointState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wearable_robot_interfaces__msg__JointState);
    wearable_robot_interfaces__msg__JointState * data =
      (wearable_robot_interfaces__msg__JointState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wearable_robot_interfaces__msg__JointState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          wearable_robot_interfaces__msg__JointState__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wearable_robot_interfaces__msg__JointState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
