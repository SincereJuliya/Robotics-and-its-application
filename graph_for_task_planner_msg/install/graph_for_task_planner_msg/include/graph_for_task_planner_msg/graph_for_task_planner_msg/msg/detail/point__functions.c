// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from graph_for_task_planner_msg:msg/Point.idl
// generated code does not contain a copyright notice
#include "graph_for_task_planner_msg/msg/detail/point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
graph_for_task_planner_msg__msg__Point__init(graph_for_task_planner_msg__msg__Point * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
graph_for_task_planner_msg__msg__Point__fini(graph_for_task_planner_msg__msg__Point * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
graph_for_task_planner_msg__msg__Point__are_equal(const graph_for_task_planner_msg__msg__Point * lhs, const graph_for_task_planner_msg__msg__Point * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
graph_for_task_planner_msg__msg__Point__copy(
  const graph_for_task_planner_msg__msg__Point * input,
  graph_for_task_planner_msg__msg__Point * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

graph_for_task_planner_msg__msg__Point *
graph_for_task_planner_msg__msg__Point__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Point * msg = (graph_for_task_planner_msg__msg__Point *)allocator.allocate(sizeof(graph_for_task_planner_msg__msg__Point), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(graph_for_task_planner_msg__msg__Point));
  bool success = graph_for_task_planner_msg__msg__Point__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
graph_for_task_planner_msg__msg__Point__destroy(graph_for_task_planner_msg__msg__Point * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    graph_for_task_planner_msg__msg__Point__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
graph_for_task_planner_msg__msg__Point__Sequence__init(graph_for_task_planner_msg__msg__Point__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Point * data = NULL;

  if (size) {
    data = (graph_for_task_planner_msg__msg__Point *)allocator.zero_allocate(size, sizeof(graph_for_task_planner_msg__msg__Point), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = graph_for_task_planner_msg__msg__Point__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        graph_for_task_planner_msg__msg__Point__fini(&data[i - 1]);
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
graph_for_task_planner_msg__msg__Point__Sequence__fini(graph_for_task_planner_msg__msg__Point__Sequence * array)
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
      graph_for_task_planner_msg__msg__Point__fini(&array->data[i]);
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

graph_for_task_planner_msg__msg__Point__Sequence *
graph_for_task_planner_msg__msg__Point__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Point__Sequence * array = (graph_for_task_planner_msg__msg__Point__Sequence *)allocator.allocate(sizeof(graph_for_task_planner_msg__msg__Point__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = graph_for_task_planner_msg__msg__Point__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
graph_for_task_planner_msg__msg__Point__Sequence__destroy(graph_for_task_planner_msg__msg__Point__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    graph_for_task_planner_msg__msg__Point__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
graph_for_task_planner_msg__msg__Point__Sequence__are_equal(const graph_for_task_planner_msg__msg__Point__Sequence * lhs, const graph_for_task_planner_msg__msg__Point__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!graph_for_task_planner_msg__msg__Point__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
graph_for_task_planner_msg__msg__Point__Sequence__copy(
  const graph_for_task_planner_msg__msg__Point__Sequence * input,
  graph_for_task_planner_msg__msg__Point__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(graph_for_task_planner_msg__msg__Point);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    graph_for_task_planner_msg__msg__Point * data =
      (graph_for_task_planner_msg__msg__Point *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!graph_for_task_planner_msg__msg__Point__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          graph_for_task_planner_msg__msg__Point__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!graph_for_task_planner_msg__msg__Point__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
