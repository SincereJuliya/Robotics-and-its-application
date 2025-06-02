// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice
#include "graph_for_task_planner_msg/msg/detail/edge__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `start_point`
// Member `end_point`
#include "graph_for_task_planner_msg/msg/detail/point__functions.h"

bool
graph_for_task_planner_msg__msg__Edge__init(graph_for_task_planner_msg__msg__Edge * msg)
{
  if (!msg) {
    return false;
  }
  // start_point
  if (!graph_for_task_planner_msg__msg__Point__init(&msg->start_point)) {
    graph_for_task_planner_msg__msg__Edge__fini(msg);
    return false;
  }
  // end_point
  if (!graph_for_task_planner_msg__msg__Point__init(&msg->end_point)) {
    graph_for_task_planner_msg__msg__Edge__fini(msg);
    return false;
  }
  return true;
}

void
graph_for_task_planner_msg__msg__Edge__fini(graph_for_task_planner_msg__msg__Edge * msg)
{
  if (!msg) {
    return;
  }
  // start_point
  graph_for_task_planner_msg__msg__Point__fini(&msg->start_point);
  // end_point
  graph_for_task_planner_msg__msg__Point__fini(&msg->end_point);
}

bool
graph_for_task_planner_msg__msg__Edge__are_equal(const graph_for_task_planner_msg__msg__Edge * lhs, const graph_for_task_planner_msg__msg__Edge * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start_point
  if (!graph_for_task_planner_msg__msg__Point__are_equal(
      &(lhs->start_point), &(rhs->start_point)))
  {
    return false;
  }
  // end_point
  if (!graph_for_task_planner_msg__msg__Point__are_equal(
      &(lhs->end_point), &(rhs->end_point)))
  {
    return false;
  }
  return true;
}

bool
graph_for_task_planner_msg__msg__Edge__copy(
  const graph_for_task_planner_msg__msg__Edge * input,
  graph_for_task_planner_msg__msg__Edge * output)
{
  if (!input || !output) {
    return false;
  }
  // start_point
  if (!graph_for_task_planner_msg__msg__Point__copy(
      &(input->start_point), &(output->start_point)))
  {
    return false;
  }
  // end_point
  if (!graph_for_task_planner_msg__msg__Point__copy(
      &(input->end_point), &(output->end_point)))
  {
    return false;
  }
  return true;
}

graph_for_task_planner_msg__msg__Edge *
graph_for_task_planner_msg__msg__Edge__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Edge * msg = (graph_for_task_planner_msg__msg__Edge *)allocator.allocate(sizeof(graph_for_task_planner_msg__msg__Edge), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(graph_for_task_planner_msg__msg__Edge));
  bool success = graph_for_task_planner_msg__msg__Edge__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
graph_for_task_planner_msg__msg__Edge__destroy(graph_for_task_planner_msg__msg__Edge * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    graph_for_task_planner_msg__msg__Edge__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
graph_for_task_planner_msg__msg__Edge__Sequence__init(graph_for_task_planner_msg__msg__Edge__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Edge * data = NULL;

  if (size) {
    data = (graph_for_task_planner_msg__msg__Edge *)allocator.zero_allocate(size, sizeof(graph_for_task_planner_msg__msg__Edge), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = graph_for_task_planner_msg__msg__Edge__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        graph_for_task_planner_msg__msg__Edge__fini(&data[i - 1]);
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
graph_for_task_planner_msg__msg__Edge__Sequence__fini(graph_for_task_planner_msg__msg__Edge__Sequence * array)
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
      graph_for_task_planner_msg__msg__Edge__fini(&array->data[i]);
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

graph_for_task_planner_msg__msg__Edge__Sequence *
graph_for_task_planner_msg__msg__Edge__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  graph_for_task_planner_msg__msg__Edge__Sequence * array = (graph_for_task_planner_msg__msg__Edge__Sequence *)allocator.allocate(sizeof(graph_for_task_planner_msg__msg__Edge__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = graph_for_task_planner_msg__msg__Edge__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
graph_for_task_planner_msg__msg__Edge__Sequence__destroy(graph_for_task_planner_msg__msg__Edge__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    graph_for_task_planner_msg__msg__Edge__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
graph_for_task_planner_msg__msg__Edge__Sequence__are_equal(const graph_for_task_planner_msg__msg__Edge__Sequence * lhs, const graph_for_task_planner_msg__msg__Edge__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!graph_for_task_planner_msg__msg__Edge__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
graph_for_task_planner_msg__msg__Edge__Sequence__copy(
  const graph_for_task_planner_msg__msg__Edge__Sequence * input,
  graph_for_task_planner_msg__msg__Edge__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(graph_for_task_planner_msg__msg__Edge);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    graph_for_task_planner_msg__msg__Edge * data =
      (graph_for_task_planner_msg__msg__Edge *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!graph_for_task_planner_msg__msg__Edge__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          graph_for_task_planner_msg__msg__Edge__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!graph_for_task_planner_msg__msg__Edge__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
