// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__TRAITS_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_for_task_planner_msg/msg/detail/edge__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "graph_for_task_planner_msg/msg/detail/point__traits.hpp"

namespace graph_for_task_planner_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Edge & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_point
  {
    out << "start_point: ";
    to_flow_style_yaml(msg.start_point, out);
    out << ", ";
  }

  // member: end_point
  {
    out << "end_point: ";
    to_flow_style_yaml(msg.end_point, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Edge & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_point:\n";
    to_block_style_yaml(msg.start_point, out, indentation + 2);
  }

  // member: end_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_point:\n";
    to_block_style_yaml(msg.end_point, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Edge & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace graph_for_task_planner_msg

namespace rosidl_generator_traits
{

[[deprecated("use graph_for_task_planner_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graph_for_task_planner_msg::msg::Edge & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_for_task_planner_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_for_task_planner_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_for_task_planner_msg::msg::Edge & msg)
{
  return graph_for_task_planner_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_for_task_planner_msg::msg::Edge>()
{
  return "graph_for_task_planner_msg::msg::Edge";
}

template<>
inline const char * name<graph_for_task_planner_msg::msg::Edge>()
{
  return "graph_for_task_planner_msg/msg/Edge";
}

template<>
struct has_fixed_size<graph_for_task_planner_msg::msg::Edge>
  : std::integral_constant<bool, has_fixed_size<graph_for_task_planner_msg::msg::Point>::value> {};

template<>
struct has_bounded_size<graph_for_task_planner_msg::msg::Edge>
  : std::integral_constant<bool, has_bounded_size<graph_for_task_planner_msg::msg::Point>::value> {};

template<>
struct is_message<graph_for_task_planner_msg::msg::Edge>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__TRAITS_HPP_
