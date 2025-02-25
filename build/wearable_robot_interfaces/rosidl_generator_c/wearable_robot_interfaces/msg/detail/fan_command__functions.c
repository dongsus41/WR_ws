// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wearable_robot_interfaces:msg/FanCommand.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/fan_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `fan`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
wearable_robot_interfaces__msg__FanCommand__init(wearable_robot_interfaces__msg__FanCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wearable_robot_interfaces__msg__FanCommand__fini(msg);
    return false;
  }
  // fan
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->fan, 0)) {
    wearable_robot_interfaces__msg__FanCommand__fini(msg);
    return false;
  }
  return true;
}

void
wearable_robot_interfaces__msg__FanCommand__fini(wearable_robot_interfaces__msg__FanCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fan
  rosidl_runtime_c__boolean__Sequence__fini(&msg->fan);
}

bool
wearable_robot_interfaces__msg__FanCommand__are_equal(const wearable_robot_interfaces__msg__FanCommand * lhs, const wearable_robot_interfaces__msg__FanCommand * rhs)
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
  // fan
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->fan), &(rhs->fan)))
  {
    return false;
  }
  return true;
}

bool
wearable_robot_interfaces__msg__FanCommand__copy(
  const wearable_robot_interfaces__msg__FanCommand * input,
  wearable_robot_interfaces__msg__FanCommand * output)
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
  // fan
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->fan), &(output->fan)))
  {
    return false;
  }
  return true;
}

wearable_robot_interfaces__msg__FanCommand *
wearable_robot_interfaces__msg__FanCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__FanCommand * msg = (wearable_robot_interfaces__msg__FanCommand *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__FanCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wearable_robot_interfaces__msg__FanCommand));
  bool success = wearable_robot_interfaces__msg__FanCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wearable_robot_interfaces__msg__FanCommand__destroy(wearable_robot_interfaces__msg__FanCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wearable_robot_interfaces__msg__FanCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wearable_robot_interfaces__msg__FanCommand__Sequence__init(wearable_robot_interfaces__msg__FanCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__FanCommand * data = NULL;

  if (size) {
    data = (wearable_robot_interfaces__msg__FanCommand *)allocator.zero_allocate(size, sizeof(wearable_robot_interfaces__msg__FanCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wearable_robot_interfaces__msg__FanCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wearable_robot_interfaces__msg__FanCommand__fini(&data[i - 1]);
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
wearable_robot_interfaces__msg__FanCommand__Sequence__fini(wearable_robot_interfaces__msg__FanCommand__Sequence * array)
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
      wearable_robot_interfaces__msg__FanCommand__fini(&array->data[i]);
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

wearable_robot_interfaces__msg__FanCommand__Sequence *
wearable_robot_interfaces__msg__FanCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__FanCommand__Sequence * array = (wearable_robot_interfaces__msg__FanCommand__Sequence *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__FanCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wearable_robot_interfaces__msg__FanCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wearable_robot_interfaces__msg__FanCommand__Sequence__destroy(wearable_robot_interfaces__msg__FanCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wearable_robot_interfaces__msg__FanCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wearable_robot_interfaces__msg__FanCommand__Sequence__are_equal(const wearable_robot_interfaces__msg__FanCommand__Sequence * lhs, const wearable_robot_interfaces__msg__FanCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wearable_robot_interfaces__msg__FanCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wearable_robot_interfaces__msg__FanCommand__Sequence__copy(
  const wearable_robot_interfaces__msg__FanCommand__Sequence * input,
  wearable_robot_interfaces__msg__FanCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wearable_robot_interfaces__msg__FanCommand);
    wearable_robot_interfaces__msg__FanCommand * data =
      (wearable_robot_interfaces__msg__FanCommand *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wearable_robot_interfaces__msg__FanCommand__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          wearable_robot_interfaces__msg__FanCommand__fini(&data[i]);
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
    if (!wearable_robot_interfaces__msg__FanCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
