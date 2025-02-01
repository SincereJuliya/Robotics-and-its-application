// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_for_task_planner_msg:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__BUILDER_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_for_task_planner_msg/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_for_task_planner_msg
{

namespace msg
{

namespace builder
{

class Init_Point_y
{
public:
  explicit Init_Point_y(::graph_for_task_planner_msg::msg::Point & msg)
  : msg_(msg)
  {}
  ::graph_for_task_planner_msg::msg::Point y(::graph_for_task_planner_msg::msg::Point::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Point msg_;
};

class Init_Point_x
{
public:
  Init_Point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_y x(::graph_for_task_planner_msg::msg::Point::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point_y(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_for_task_planner_msg::msg::Point>()
{
  return graph_for_task_planner_msg::msg::builder::Init_Point_x();
}

}  // namespace graph_for_task_planner_msg

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__BUILDER_HPP_
