// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_for_task_planner_msg:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__TRAITS_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_for_task_planner_msg/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace graph_for_task_planner_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Point & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Point & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Point & msg, bool use_flow_style = false)
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
  const graph_for_task_planner_msg::msg::Point & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_for_task_planner_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_for_task_planner_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_for_task_planner_msg::msg::Point & msg)
{
  return graph_for_task_planner_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_for_task_planner_msg::msg::Point>()
{
  return "graph_for_task_planner_msg::msg::Point";
}

template<>
inline const char * name<graph_for_task_planner_msg::msg::Point>()
{
  return "graph_for_task_planner_msg/msg/Point";
}

template<>
struct has_fixed_size<graph_for_task_planner_msg::msg::Point>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<graph_for_task_planner_msg::msg::Point>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<graph_for_task_planner_msg::msg::Point>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__POINT__TRAITS_HPP_
