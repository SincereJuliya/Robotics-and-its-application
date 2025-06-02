// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_for_task_planner_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_for_task_planner_msg/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vertices'
#include "graph_for_task_planner_msg/msg/detail/point__traits.hpp"
// Member 'edges'
#include "graph_for_task_planner_msg/msg/detail/edge__traits.hpp"

namespace graph_for_task_planner_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Graph & msg,
  std::ostream & out)
{
  out << "{";
  // member: vertices
  {
    if (msg.vertices.size() == 0) {
      out << "vertices: []";
    } else {
      out << "vertices: [";
      size_t pending_items = msg.vertices.size();
      for (auto item : msg.vertices) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: edges
  {
    if (msg.edges.size() == 0) {
      out << "edges: []";
    } else {
      out << "edges: [";
      size_t pending_items = msg.edges.size();
      for (auto item : msg.edges) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vertices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vertices.size() == 0) {
      out << "vertices: []\n";
    } else {
      out << "vertices:\n";
      for (auto item : msg.vertices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.edges.size() == 0) {
      out << "edges: []\n";
    } else {
      out << "edges:\n";
      for (auto item : msg.edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Graph & msg, bool use_flow_style = false)
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
  const graph_for_task_planner_msg::msg::Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_for_task_planner_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_for_task_planner_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_for_task_planner_msg::msg::Graph & msg)
{
  return graph_for_task_planner_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_for_task_planner_msg::msg::Graph>()
{
  return "graph_for_task_planner_msg::msg::Graph";
}

template<>
inline const char * name<graph_for_task_planner_msg::msg::Graph>()
{
  return "graph_for_task_planner_msg/msg/Graph";
}

template<>
struct has_fixed_size<graph_for_task_planner_msg::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<graph_for_task_planner_msg::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<graph_for_task_planner_msg::msg::Graph>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_
