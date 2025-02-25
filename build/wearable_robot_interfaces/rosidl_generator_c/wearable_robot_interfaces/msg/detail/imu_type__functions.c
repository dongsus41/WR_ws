// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/imu_type__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
wearable_robot_interfaces__msg__IMUType__init(wearable_robot_interfaces__msg__IMUType * msg)
{
  if (!msg) {
    return false;
  }
  // roll
  // pitch
  // yaw
  return true;
}

void
wearable_robot_interfaces__msg__IMUType__fini(wearable_robot_interfaces__msg__IMUType * msg)
{
  if (!msg) {
    return;
  }
  // roll
  // pitch
  // yaw
}

bool
wearable_robot_interfaces__msg__IMUType__are_equal(const wearable_robot_interfaces__msg__IMUType * lhs, const wearable_robot_interfaces__msg__IMUType * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
wearable_robot_interfaces__msg__IMUType__copy(
  const wearable_robot_interfaces__msg__IMUType * input,
  wearable_robot_interfaces__msg__IMUType * output)
{
  if (!input || !output) {
    return false;
  }
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

wearable_robot_interfaces__msg__IMUType *
wearable_robot_interfaces__msg__IMUType__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMUType * msg = (wearable_robot_interfaces__msg__IMUType *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__IMUType), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wearable_robot_interfaces__msg__IMUType));
  bool success = wearable_robot_interfaces__msg__IMUType__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wearable_robot_interfaces__msg__IMUType__destroy(wearable_robot_interfaces__msg__IMUType * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wearable_robot_interfaces__msg__IMUType__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wearable_robot_interfaces__msg__IMUType__Sequence__init(wearable_robot_interfaces__msg__IMUType__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMUType * data = NULL;

  if (size) {
    data = (wearable_robot_interfaces__msg__IMUType *)allocator.zero_allocate(size, sizeof(wearable_robot_interfaces__msg__IMUType), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wearable_robot_interfaces__msg__IMUType__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wearable_robot_interfaces__msg__IMUType__fini(&data[i - 1]);
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
wearable_robot_interfaces__msg__IMUType__Sequence__fini(wearable_robot_interfaces__msg__IMUType__Sequence * array)
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
      wearable_robot_interfaces__msg__IMUType__fini(&array->data[i]);
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

wearable_robot_interfaces__msg__IMUType__Sequence *
wearable_robot_interfaces__msg__IMUType__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMUType__Sequence * array = (wearable_robot_interfaces__msg__IMUType__Sequence *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__IMUType__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wearable_robot_interfaces__msg__IMUType__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wearable_robot_interfaces__msg__IMUType__Sequence__destroy(wearable_robot_interfaces__msg__IMUType__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wearable_robot_interfaces__msg__IMUType__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wearable_robot_interfaces__msg__IMUType__Sequence__are_equal(const wearable_robot_interfaces__msg__IMUType__Sequence * lhs, const wearable_robot_interfaces__msg__IMUType__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wearable_robot_interfaces__msg__IMUType__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wearable_robot_interfaces__msg__IMUType__Sequence__copy(
  const wearable_robot_interfaces__msg__IMUType__Sequence * input,
  wearable_robot_interfaces__msg__IMUType__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wearable_robot_interfaces__msg__IMUType);
    wearable_robot_interfaces__msg__IMUType * data =
      (wearable_robot_interfaces__msg__IMUType *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wearable_robot_interfaces__msg__IMUType__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          wearable_robot_interfaces__msg__IMUType__fini(&data[i]);
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
    if (!wearable_robot_interfaces__msg__IMUType__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
