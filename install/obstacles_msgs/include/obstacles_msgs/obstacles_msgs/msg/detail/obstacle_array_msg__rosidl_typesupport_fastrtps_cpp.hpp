// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from obstacles_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "obstacles_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "obstacles_msgs/msg/detail/obstacle_array_msg__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace obstacles_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_obstacles_msgs
cdr_serialize(
  const obstacles_msgs::msg::ObstacleArrayMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_obstacles_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  obstacles_msgs::msg::ObstacleArrayMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_obstacles_msgs
get_serialized_size(
  const obstacles_msgs::msg::ObstacleArrayMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_obstacles_msgs
max_serialized_size_ObstacleArrayMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace obstacles_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_obstacles_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, obstacles_msgs, msg, ObstacleArrayMsg)();

#ifdef __cplusplus
}
#endif

#endif  // OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
