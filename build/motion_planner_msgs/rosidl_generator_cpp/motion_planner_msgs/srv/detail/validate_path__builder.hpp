// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motion_planner_msgs:srv/ValidatePath.idl
// generated code does not contain a copyright notice

#ifndef MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__BUILDER_HPP_
#define MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motion_planner_msgs/srv/detail/validate_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motion_planner_msgs
{

namespace srv
{

namespace builder
{

class Init_ValidatePath_Request_path
{
public:
  Init_ValidatePath_Request_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::motion_planner_msgs::srv::ValidatePath_Request path(::motion_planner_msgs::srv::ValidatePath_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motion_planner_msgs::srv::ValidatePath_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motion_planner_msgs::srv::ValidatePath_Request>()
{
  return motion_planner_msgs::srv::builder::Init_ValidatePath_Request_path();
}

}  // namespace motion_planner_msgs


namespace motion_planner_msgs
{

namespace srv
{

namespace builder
{

class Init_ValidatePath_Response_valid
{
public:
  Init_ValidatePath_Response_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::motion_planner_msgs::srv::ValidatePath_Response valid(::motion_planner_msgs::srv::ValidatePath_Response::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motion_planner_msgs::srv::ValidatePath_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motion_planner_msgs::srv::ValidatePath_Response>()
{
  return motion_planner_msgs::srv::builder::Init_ValidatePath_Response_valid();
}

}  // namespace motion_planner_msgs

#endif  // MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__BUILDER_HPP_
