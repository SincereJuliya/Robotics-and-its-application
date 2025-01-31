// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_for_task_planner_msg:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__STRUCT_H_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Point in the package graph_for_task_planner_msg.
typedef struct graph_for_task_planner_msg__msg__Point
{
  double x;
  double y;
} graph_for_task_planner_msg__msg__Point;

// Struct for a sequence of graph_for_task_planner_msg__msg__Point.
typedef struct graph_for_task_planner_msg__msg__Point__Sequence
{
  graph_for_task_planner_msg__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_for_task_planner_msg__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__STRUCT_H_
