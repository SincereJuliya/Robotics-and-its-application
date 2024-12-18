// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from obstacles_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice
#include "obstacles_msgs/msg/detail/obstacle_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `polygon`
#include "geometry_msgs/msg/detail/polygon__functions.h"

bool
obstacles_msgs__msg__ObstacleMsg__init(obstacles_msgs__msg__ObstacleMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    obstacles_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  // polygon
  if (!geometry_msgs__msg__Polygon__init(&msg->polygon)) {
    obstacles_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  // radius
  return true;
}

void
obstacles_msgs__msg__ObstacleMsg__fini(obstacles_msgs__msg__ObstacleMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // polygon
  geometry_msgs__msg__Polygon__fini(&msg->polygon);
  // radius
}

bool
obstacles_msgs__msg__ObstacleMsg__are_equal(const obstacles_msgs__msg__ObstacleMsg * lhs, const obstacles_msgs__msg__ObstacleMsg * rhs)
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
  // polygon
  if (!geometry_msgs__msg__Polygon__are_equal(
      &(lhs->polygon), &(rhs->polygon)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  return true;
}

bool
obstacles_msgs__msg__ObstacleMsg__copy(
  const obstacles_msgs__msg__ObstacleMsg * input,
  obstacles_msgs__msg__ObstacleMsg * output)
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
  // polygon
  if (!geometry_msgs__msg__Polygon__copy(
      &(input->polygon), &(output->polygon)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  return true;
}

obstacles_msgs__msg__ObstacleMsg *
obstacles_msgs__msg__ObstacleMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacles_msgs__msg__ObstacleMsg * msg = (obstacles_msgs__msg__ObstacleMsg *)allocator.allocate(sizeof(obstacles_msgs__msg__ObstacleMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(obstacles_msgs__msg__ObstacleMsg));
  bool success = obstacles_msgs__msg__ObstacleMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
obstacles_msgs__msg__ObstacleMsg__destroy(obstacles_msgs__msg__ObstacleMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    obstacles_msgs__msg__ObstacleMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
obstacles_msgs__msg__ObstacleMsg__Sequence__init(obstacles_msgs__msg__ObstacleMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacles_msgs__msg__ObstacleMsg * data = NULL;

  if (size) {
    data = (obstacles_msgs__msg__ObstacleMsg *)allocator.zero_allocate(size, sizeof(obstacles_msgs__msg__ObstacleMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = obstacles_msgs__msg__ObstacleMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        obstacles_msgs__msg__ObstacleMsg__fini(&data[i - 1]);
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
obstacles_msgs__msg__ObstacleMsg__Sequence__fini(obstacles_msgs__msg__ObstacleMsg__Sequence * array)
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
      obstacles_msgs__msg__ObstacleMsg__fini(&array->data[i]);
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

obstacles_msgs__msg__ObstacleMsg__Sequence *
obstacles_msgs__msg__ObstacleMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  obstacles_msgs__msg__ObstacleMsg__Sequence * array = (obstacles_msgs__msg__ObstacleMsg__Sequence *)allocator.allocate(sizeof(obstacles_msgs__msg__ObstacleMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = obstacles_msgs__msg__ObstacleMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
obstacles_msgs__msg__ObstacleMsg__Sequence__destroy(obstacles_msgs__msg__ObstacleMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    obstacles_msgs__msg__ObstacleMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
obstacles_msgs__msg__ObstacleMsg__Sequence__are_equal(const obstacles_msgs__msg__ObstacleMsg__Sequence * lhs, const obstacles_msgs__msg__ObstacleMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!obstacles_msgs__msg__ObstacleMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
obstacles_msgs__msg__ObstacleMsg__Sequence__copy(
  const obstacles_msgs__msg__ObstacleMsg__Sequence * input,
  obstacles_msgs__msg__ObstacleMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(obstacles_msgs__msg__ObstacleMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    obstacles_msgs__msg__ObstacleMsg * data =
      (obstacles_msgs__msg__ObstacleMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!obstacles_msgs__msg__ObstacleMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          obstacles_msgs__msg__ObstacleMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!obstacles_msgs__msg__ObstacleMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
