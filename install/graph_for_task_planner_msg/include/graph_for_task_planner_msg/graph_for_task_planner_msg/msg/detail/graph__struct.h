// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_for_task_planner_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__STRUCT_H_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vertices'
#include "graph_for_task_planner_msg/msg/detail/point__struct.h"
// Member 'edges'
#include "graph_for_task_planner_msg/msg/detail/edge__struct.h"

/// Struct defined in msg/Graph in the package graph_for_task_planner_msg.
typedef struct graph_for_task_planner_msg__msg__Graph
{
  graph_for_task_planner_msg__msg__Point__Sequence vertices;
  graph_for_task_planner_msg__msg__Edge__Sequence edges;
} graph_for_task_planner_msg__msg__Graph;

// Struct for a sequence of graph_for_task_planner_msg__msg__Graph.
typedef struct graph_for_task_planner_msg__msg__Graph__Sequence
{
  graph_for_task_planner_msg__msg__Graph * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_for_task_planner_msg__msg__Graph__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__STRUCT_H_
