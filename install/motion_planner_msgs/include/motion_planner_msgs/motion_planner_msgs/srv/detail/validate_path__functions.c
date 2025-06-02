// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motion_planner_msgs:srv/ValidatePath.idl
// generated code does not contain a copyright notice
#include "motion_planner_msgs/srv/detail/validate_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `path`
#include "geometry_msgs/msg/detail/pose_array__functions.h"

bool
motion_planner_msgs__srv__ValidatePath_Request__init(motion_planner_msgs__srv__ValidatePath_Request * msg)
{
  if (!msg) {
    return false;
  }
  // path
  if (!geometry_msgs__msg__PoseArray__init(&msg->path)) {
    motion_planner_msgs__srv__ValidatePath_Request__fini(msg);
    return false;
  }
  return true;
}

void
motion_planner_msgs__srv__ValidatePath_Request__fini(motion_planner_msgs__srv__ValidatePath_Request * msg)
{
  if (!msg) {
    return;
  }
  // path
  geometry_msgs__msg__PoseArray__fini(&msg->path);
}

bool
motion_planner_msgs__srv__ValidatePath_Request__are_equal(const motion_planner_msgs__srv__ValidatePath_Request * lhs, const motion_planner_msgs__srv__ValidatePath_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path
  if (!geometry_msgs__msg__PoseArray__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
motion_planner_msgs__srv__ValidatePath_Request__copy(
  const motion_planner_msgs__srv__ValidatePath_Request * input,
  motion_planner_msgs__srv__ValidatePath_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // path
  if (!geometry_msgs__msg__PoseArray__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

motion_planner_msgs__srv__ValidatePath_Request *
motion_planner_msgs__srv__ValidatePath_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Request * msg = (motion_planner_msgs__srv__ValidatePath_Request *)allocator.allocate(sizeof(motion_planner_msgs__srv__ValidatePath_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_planner_msgs__srv__ValidatePath_Request));
  bool success = motion_planner_msgs__srv__ValidatePath_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_planner_msgs__srv__ValidatePath_Request__destroy(motion_planner_msgs__srv__ValidatePath_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_planner_msgs__srv__ValidatePath_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__init(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Request * data = NULL;

  if (size) {
    data = (motion_planner_msgs__srv__ValidatePath_Request *)allocator.zero_allocate(size, sizeof(motion_planner_msgs__srv__ValidatePath_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_planner_msgs__srv__ValidatePath_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_planner_msgs__srv__ValidatePath_Request__fini(&data[i - 1]);
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
motion_planner_msgs__srv__ValidatePath_Request__Sequence__fini(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array)
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
      motion_planner_msgs__srv__ValidatePath_Request__fini(&array->data[i]);
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

motion_planner_msgs__srv__ValidatePath_Request__Sequence *
motion_planner_msgs__srv__ValidatePath_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Request__Sequence * array = (motion_planner_msgs__srv__ValidatePath_Request__Sequence *)allocator.allocate(sizeof(motion_planner_msgs__srv__ValidatePath_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_planner_msgs__srv__ValidatePath_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_planner_msgs__srv__ValidatePath_Request__Sequence__destroy(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_planner_msgs__srv__ValidatePath_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__are_equal(const motion_planner_msgs__srv__ValidatePath_Request__Sequence * lhs, const motion_planner_msgs__srv__ValidatePath_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_planner_msgs__srv__ValidatePath_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__copy(
  const motion_planner_msgs__srv__ValidatePath_Request__Sequence * input,
  motion_planner_msgs__srv__ValidatePath_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_planner_msgs__srv__ValidatePath_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_planner_msgs__srv__ValidatePath_Request * data =
      (motion_planner_msgs__srv__ValidatePath_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_planner_msgs__srv__ValidatePath_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_planner_msgs__srv__ValidatePath_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_planner_msgs__srv__ValidatePath_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
motion_planner_msgs__srv__ValidatePath_Response__init(motion_planner_msgs__srv__ValidatePath_Response * msg)
{
  if (!msg) {
    return false;
  }
  // valid
  return true;
}

void
motion_planner_msgs__srv__ValidatePath_Response__fini(motion_planner_msgs__srv__ValidatePath_Response * msg)
{
  if (!msg) {
    return;
  }
  // valid
}

bool
motion_planner_msgs__srv__ValidatePath_Response__are_equal(const motion_planner_msgs__srv__ValidatePath_Response * lhs, const motion_planner_msgs__srv__ValidatePath_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  return true;
}

bool
motion_planner_msgs__srv__ValidatePath_Response__copy(
  const motion_planner_msgs__srv__ValidatePath_Response * input,
  motion_planner_msgs__srv__ValidatePath_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // valid
  output->valid = input->valid;
  return true;
}

motion_planner_msgs__srv__ValidatePath_Response *
motion_planner_msgs__srv__ValidatePath_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Response * msg = (motion_planner_msgs__srv__ValidatePath_Response *)allocator.allocate(sizeof(motion_planner_msgs__srv__ValidatePath_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_planner_msgs__srv__ValidatePath_Response));
  bool success = motion_planner_msgs__srv__ValidatePath_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_planner_msgs__srv__ValidatePath_Response__destroy(motion_planner_msgs__srv__ValidatePath_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_planner_msgs__srv__ValidatePath_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__init(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Response * data = NULL;

  if (size) {
    data = (motion_planner_msgs__srv__ValidatePath_Response *)allocator.zero_allocate(size, sizeof(motion_planner_msgs__srv__ValidatePath_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_planner_msgs__srv__ValidatePath_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_planner_msgs__srv__ValidatePath_Response__fini(&data[i - 1]);
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
motion_planner_msgs__srv__ValidatePath_Response__Sequence__fini(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array)
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
      motion_planner_msgs__srv__ValidatePath_Response__fini(&array->data[i]);
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

motion_planner_msgs__srv__ValidatePath_Response__Sequence *
motion_planner_msgs__srv__ValidatePath_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_planner_msgs__srv__ValidatePath_Response__Sequence * array = (motion_planner_msgs__srv__ValidatePath_Response__Sequence *)allocator.allocate(sizeof(motion_planner_msgs__srv__ValidatePath_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_planner_msgs__srv__ValidatePath_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_planner_msgs__srv__ValidatePath_Response__Sequence__destroy(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_planner_msgs__srv__ValidatePath_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__are_equal(const motion_planner_msgs__srv__ValidatePath_Response__Sequence * lhs, const motion_planner_msgs__srv__ValidatePath_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_planner_msgs__srv__ValidatePath_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__copy(
  const motion_planner_msgs__srv__ValidatePath_Response__Sequence * input,
  motion_planner_msgs__srv__ValidatePath_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_planner_msgs__srv__ValidatePath_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_planner_msgs__srv__ValidatePath_Response * data =
      (motion_planner_msgs__srv__ValidatePath_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_planner_msgs__srv__ValidatePath_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_planner_msgs__srv__ValidatePath_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_planner_msgs__srv__ValidatePath_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
