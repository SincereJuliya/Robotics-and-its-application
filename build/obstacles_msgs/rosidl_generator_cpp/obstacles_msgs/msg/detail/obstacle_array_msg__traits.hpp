// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from obstacles_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_
#define OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "obstacles_msgs/msg/detail/obstacle_array_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'obstacles'
#include "obstacles_msgs/msg/detail/obstacle_msg__traits.hpp"

namespace obstacles_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleArrayMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: obstacles
  {
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []";
    } else {
      out << "obstacles: [";
      size_t pending_items = msg.obstacles.size();
      for (auto item : msg.obstacles) {
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
  const ObstacleArrayMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: obstacles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []\n";
    } else {
      out << "obstacles:\n";
      for (auto item : msg.obstacles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleArrayMsg & msg, bool use_flow_style = false)
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

}  // namespace obstacles_msgs

namespace rosidl_generator_traits
{

[[deprecated("use obstacles_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const obstacles_msgs::msg::ObstacleArrayMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  obstacles_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use obstacles_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const obstacles_msgs::msg::ObstacleArrayMsg & msg)
{
  return obstacles_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<obstacles_msgs::msg::ObstacleArrayMsg>()
{
  return "obstacles_msgs::msg::ObstacleArrayMsg";
}

template<>
inline const char * name<obstacles_msgs::msg::ObstacleArrayMsg>()
{
  return "obstacles_msgs/msg/ObstacleArrayMsg";
}

template<>
struct has_fixed_size<obstacles_msgs::msg::ObstacleArrayMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<obstacles_msgs::msg::ObstacleArrayMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<obstacles_msgs::msg::ObstacleArrayMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__TRAITS_HPP_
