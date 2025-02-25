// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice
#include "wearable_robot_interfaces/msg/detail/imu_raw_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `imu1`
// Member `imu2`
// Member `imu3`
// Member `imu4`
#include "wearable_robot_interfaces/msg/detail/imu_type__functions.h"

bool
wearable_robot_interfaces__msg__IMURawData__init(wearable_robot_interfaces__msg__IMURawData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
    return false;
  }
  // can_id
  // imu1
  if (!wearable_robot_interfaces__msg__IMUType__init(&msg->imu1)) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
    return false;
  }
  // imu2
  if (!wearable_robot_interfaces__msg__IMUType__init(&msg->imu2)) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
    return false;
  }
  // imu3
  if (!wearable_robot_interfaces__msg__IMUType__init(&msg->imu3)) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
    return false;
  }
  // imu4
  if (!wearable_robot_interfaces__msg__IMUType__init(&msg->imu4)) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
    return false;
  }
  return true;
}

void
wearable_robot_interfaces__msg__IMURawData__fini(wearable_robot_interfaces__msg__IMURawData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // can_id
  // imu1
  wearable_robot_interfaces__msg__IMUType__fini(&msg->imu1);
  // imu2
  wearable_robot_interfaces__msg__IMUType__fini(&msg->imu2);
  // imu3
  wearable_robot_interfaces__msg__IMUType__fini(&msg->imu3);
  // imu4
  wearable_robot_interfaces__msg__IMUType__fini(&msg->imu4);
}

bool
wearable_robot_interfaces__msg__IMURawData__are_equal(const wearable_robot_interfaces__msg__IMURawData * lhs, const wearable_robot_interfaces__msg__IMURawData * rhs)
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
  // can_id
  if (lhs->can_id != rhs->can_id) {
    return false;
  }
  // imu1
  if (!wearable_robot_interfaces__msg__IMUType__are_equal(
      &(lhs->imu1), &(rhs->imu1)))
  {
    return false;
  }
  // imu2
  if (!wearable_robot_interfaces__msg__IMUType__are_equal(
      &(lhs->imu2), &(rhs->imu2)))
  {
    return false;
  }
  // imu3
  if (!wearable_robot_interfaces__msg__IMUType__are_equal(
      &(lhs->imu3), &(rhs->imu3)))
  {
    return false;
  }
  // imu4
  if (!wearable_robot_interfaces__msg__IMUType__are_equal(
      &(lhs->imu4), &(rhs->imu4)))
  {
    return false;
  }
  return true;
}

bool
wearable_robot_interfaces__msg__IMURawData__copy(
  const wearable_robot_interfaces__msg__IMURawData * input,
  wearable_robot_interfaces__msg__IMURawData * output)
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
  // can_id
  output->can_id = input->can_id;
  // imu1
  if (!wearable_robot_interfaces__msg__IMUType__copy(
      &(input->imu1), &(output->imu1)))
  {
    return false;
  }
  // imu2
  if (!wearable_robot_interfaces__msg__IMUType__copy(
      &(input->imu2), &(output->imu2)))
  {
    return false;
  }
  // imu3
  if (!wearable_robot_interfaces__msg__IMUType__copy(
      &(input->imu3), &(output->imu3)))
  {
    return false;
  }
  // imu4
  if (!wearable_robot_interfaces__msg__IMUType__copy(
      &(input->imu4), &(output->imu4)))
  {
    return false;
  }
  return true;
}

wearable_robot_interfaces__msg__IMURawData *
wearable_robot_interfaces__msg__IMURawData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMURawData * msg = (wearable_robot_interfaces__msg__IMURawData *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__IMURawData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wearable_robot_interfaces__msg__IMURawData));
  bool success = wearable_robot_interfaces__msg__IMURawData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wearable_robot_interfaces__msg__IMURawData__destroy(wearable_robot_interfaces__msg__IMURawData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wearable_robot_interfaces__msg__IMURawData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wearable_robot_interfaces__msg__IMURawData__Sequence__init(wearable_robot_interfaces__msg__IMURawData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMURawData * data = NULL;

  if (size) {
    data = (wearable_robot_interfaces__msg__IMURawData *)allocator.zero_allocate(size, sizeof(wearable_robot_interfaces__msg__IMURawData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wearable_robot_interfaces__msg__IMURawData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wearable_robot_interfaces__msg__IMURawData__fini(&data[i - 1]);
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
wearable_robot_interfaces__msg__IMURawData__Sequence__fini(wearable_robot_interfaces__msg__IMURawData__Sequence * array)
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
      wearable_robot_interfaces__msg__IMURawData__fini(&array->data[i]);
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

wearable_robot_interfaces__msg__IMURawData__Sequence *
wearable_robot_interfaces__msg__IMURawData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wearable_robot_interfaces__msg__IMURawData__Sequence * array = (wearable_robot_interfaces__msg__IMURawData__Sequence *)allocator.allocate(sizeof(wearable_robot_interfaces__msg__IMURawData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wearable_robot_interfaces__msg__IMURawData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wearable_robot_interfaces__msg__IMURawData__Sequence__destroy(wearable_robot_interfaces__msg__IMURawData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wearable_robot_interfaces__msg__IMURawData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wearable_robot_interfaces__msg__IMURawData__Sequence__are_equal(const wearable_robot_interfaces__msg__IMURawData__Sequence * lhs, const wearable_robot_interfaces__msg__IMURawData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wearable_robot_interfaces__msg__IMURawData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wearable_robot_interfaces__msg__IMURawData__Sequence__copy(
  const wearable_robot_interfaces__msg__IMURawData__Sequence * input,
  wearable_robot_interfaces__msg__IMURawData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wearable_robot_interfaces__msg__IMURawData);
    wearable_robot_interfaces__msg__IMURawData * data =
      (wearable_robot_interfaces__msg__IMURawData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wearable_robot_interfaces__msg__IMURawData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          wearable_robot_interfaces__msg__IMURawData__fini(&data[i]);
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
    if (!wearable_robot_interfaces__msg__IMURawData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
