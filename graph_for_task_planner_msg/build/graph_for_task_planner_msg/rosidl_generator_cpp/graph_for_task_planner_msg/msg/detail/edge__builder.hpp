// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__BUILDER_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_for_task_planner_msg/msg/detail/edge__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_for_task_planner_msg
{

namespace msg
{

namespace builder
{

class Init_Edge_end_point
{
public:
  explicit Init_Edge_end_point(::graph_for_task_planner_msg::msg::Edge & msg)
  : msg_(msg)
  {}
  ::graph_for_task_planner_msg::msg::Edge end_point(::graph_for_task_planner_msg::msg::Edge::_end_point_type arg)
  {
    msg_.end_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Edge msg_;
};

class Init_Edge_start_point
{
public:
  Init_Edge_start_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Edge_end_point start_point(::graph_for_task_planner_msg::msg::Edge::_start_point_type arg)
  {
    msg_.start_point = std::move(arg);
    return Init_Edge_end_point(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Edge msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_for_task_planner_msg::msg::Edge>()
{
  return graph_for_task_planner_msg::msg::builder::Init_Edge_start_point();
}

}  // namespace graph_for_task_planner_msg

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__BUILDER_HPP_
