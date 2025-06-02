// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motion_planner_msgs:srv/ValidatePath.idl
// generated code does not contain a copyright notice

#ifndef MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__STRUCT_H_
#define MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "geometry_msgs/msg/detail/pose_array__struct.h"

/// Struct defined in srv/ValidatePath in the package motion_planner_msgs.
typedef struct motion_planner_msgs__srv__ValidatePath_Request
{
  /// Request: Path to validate
  geometry_msgs__msg__PoseArray path;
} motion_planner_msgs__srv__ValidatePath_Request;

// Struct for a sequence of motion_planner_msgs__srv__ValidatePath_Request.
typedef struct motion_planner_msgs__srv__ValidatePath_Request__Sequence
{
  motion_planner_msgs__srv__ValidatePath_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_planner_msgs__srv__ValidatePath_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ValidatePath in the package motion_planner_msgs.
typedef struct motion_planner_msgs__srv__ValidatePath_Response
{
  bool valid;
} motion_planner_msgs__srv__ValidatePath_Response;

// Struct for a sequence of motion_planner_msgs__srv__ValidatePath_Response.
typedef struct motion_planner_msgs__srv__ValidatePath_Response__Sequence
{
  motion_planner_msgs__srv__ValidatePath_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_planner_msgs__srv__ValidatePath_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__STRUCT_H_
