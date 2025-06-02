// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motion_planner_msgs:srv/ValidatePath.idl
// generated code does not contain a copyright notice

#ifndef MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__TRAITS_HPP_
#define MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motion_planner_msgs/srv/detail/validate_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'path'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"

namespace motion_planner_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidatePath_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    out << "path: ";
    to_flow_style_yaml(msg.path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ValidatePath_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path:\n";
    to_block_style_yaml(msg.path, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ValidatePath_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace motion_planner_msgs

namespace rosidl_generator_traits
{

[[deprecated("use motion_planner_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motion_planner_msgs::srv::ValidatePath_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  motion_planner_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motion_planner_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const motion_planner_msgs::srv::ValidatePath_Request & msg)
{
  return motion_planner_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<motion_planner_msgs::srv::ValidatePath_Request>()
{
  return "motion_planner_msgs::srv::ValidatePath_Request";
}

template<>
inline const char * name<motion_planner_msgs::srv::ValidatePath_Request>()
{
  return "motion_planner_msgs/srv/ValidatePath_Request";
}

template<>
struct has_fixed_size<motion_planner_msgs::srv::ValidatePath_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseArray>::value> {};

template<>
struct has_bounded_size<motion_planner_msgs::srv::ValidatePath_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseArray>::value> {};

template<>
struct is_message<motion_planner_msgs::srv::ValidatePath_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace motion_planner_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ValidatePath_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ValidatePath_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ValidatePath_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace motion_planner_msgs

namespace rosidl_generator_traits
{

[[deprecated("use motion_planner_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motion_planner_msgs::srv::ValidatePath_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  motion_planner_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motion_planner_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const motion_planner_msgs::srv::ValidatePath_Response & msg)
{
  return motion_planner_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<motion_planner_msgs::srv::ValidatePath_Response>()
{
  return "motion_planner_msgs::srv::ValidatePath_Response";
}

template<>
inline const char * name<motion_planner_msgs::srv::ValidatePath_Response>()
{
  return "motion_planner_msgs/srv/ValidatePath_Response";
}

template<>
struct has_fixed_size<motion_planner_msgs::srv::ValidatePath_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motion_planner_msgs::srv::ValidatePath_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motion_planner_msgs::srv::ValidatePath_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<motion_planner_msgs::srv::ValidatePath>()
{
  return "motion_planner_msgs::srv::ValidatePath";
}

template<>
inline const char * name<motion_planner_msgs::srv::ValidatePath>()
{
  return "motion_planner_msgs/srv/ValidatePath";
}

template<>
struct has_fixed_size<motion_planner_msgs::srv::ValidatePath>
  : std::integral_constant<
    bool,
    has_fixed_size<motion_planner_msgs::srv::ValidatePath_Request>::value &&
    has_fixed_size<motion_planner_msgs::srv::ValidatePath_Response>::value
  >
{
};

template<>
struct has_bounded_size<motion_planner_msgs::srv::ValidatePath>
  : std::integral_constant<
    bool,
    has_bounded_size<motion_planner_msgs::srv::ValidatePath_Request>::value &&
    has_bounded_size<motion_planner_msgs::srv::ValidatePath_Response>::value
  >
{
};

template<>
struct is_service<motion_planner_msgs::srv::ValidatePath>
  : std::true_type
{
};

template<>
struct is_service_request<motion_planner_msgs::srv::ValidatePath_Request>
  : std::true_type
{
};

template<>
struct is_service_response<motion_planner_msgs::srv::ValidatePath_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__TRAITS_HPP_
