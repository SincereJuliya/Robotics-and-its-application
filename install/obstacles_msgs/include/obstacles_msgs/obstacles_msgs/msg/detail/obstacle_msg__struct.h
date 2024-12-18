// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from obstacles_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_H_
#define OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.h"

/// Struct defined in msg/ObstacleMsg in the package obstacles_msgs.
typedef struct obstacles_msgs__msg__ObstacleMsg
{
  std_msgs__msg__Header header;
  /// Obstacle footprint (polygon descriptions)
  geometry_msgs__msg__Polygon polygon;
  /// Specify the radius for circular/point obstacles
  double radius;
} obstacles_msgs__msg__ObstacleMsg;

// Struct for a sequence of obstacles_msgs__msg__ObstacleMsg.
typedef struct obstacles_msgs__msg__ObstacleMsg__Sequence
{
  obstacles_msgs__msg__ObstacleMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} obstacles_msgs__msg__ObstacleMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_H_
