// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_for_task_planner_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_for_task_planner_msg/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_for_task_planner_msg
{

namespace msg
{

namespace builder
{

class Init_Graph_edges
{
public:
  explicit Init_Graph_edges(::graph_for_task_planner_msg::msg::Graph & msg)
  : msg_(msg)
  {}
  ::graph_for_task_planner_msg::msg::Graph edges(::graph_for_task_planner_msg::msg::Graph::_edges_type arg)
  {
    msg_.edges = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Graph msg_;
};

class Init_Graph_vertices
{
public:
  Init_Graph_vertices()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Graph_edges vertices(::graph_for_task_planner_msg::msg::Graph::_vertices_type arg)
  {
    msg_.vertices = std::move(arg);
    return Init_Graph_edges(msg_);
  }

private:
  ::graph_for_task_planner_msg::msg::Graph msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_for_task_planner_msg::msg::Graph>()
{
  return graph_for_task_planner_msg::msg::builder::Init_Graph_vertices();
}

}  // namespace graph_for_task_planner_msg

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_
