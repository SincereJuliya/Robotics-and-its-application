// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from obstacles_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
#define OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "obstacles_msgs/msg/detail/obstacle_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace obstacles_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleMsg_radius
{
public:
  explicit Init_ObstacleMsg_radius(::obstacles_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  ::obstacles_msgs::msg::ObstacleMsg radius(::obstacles_msgs::msg::ObstacleMsg::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::obstacles_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_polygon
{
public:
  explicit Init_ObstacleMsg_polygon(::obstacles_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_radius polygon(::obstacles_msgs::msg::ObstacleMsg::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_ObstacleMsg_radius(msg_);
  }

private:
  ::obstacles_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_header
{
public:
  Init_ObstacleMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleMsg_polygon header(::obstacles_msgs::msg::ObstacleMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleMsg_polygon(msg_);
  }

private:
  ::obstacles_msgs::msg::ObstacleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::obstacles_msgs::msg::ObstacleMsg>()
{
  return obstacles_msgs::msg::builder::Init_ObstacleMsg_header();
}

}  // namespace obstacles_msgs

#endif  // OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
