// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_H_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "graph_for_task_planner_msg/msg/detail/point__struct.h"

/// Struct defined in msg/Edge in the package graph_for_task_planner_msg.
/**
  * Edge.msg
 */
typedef struct graph_for_task_planner_msg__msg__Edge
{
  graph_for_task_planner_msg__msg__Point start_point;
  graph_for_task_planner_msg__msg__Point end_point;
} graph_for_task_planner_msg__msg__Edge;

// Struct for a sequence of graph_for_task_planner_msg__msg__Edge.
typedef struct graph_for_task_planner_msg__msg__Edge__Sequence
{
  graph_for_task_planner_msg__msg__Edge * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_for_task_planner_msg__msg__Edge__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_H_
