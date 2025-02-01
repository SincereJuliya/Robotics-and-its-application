// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "graph_for_task_planner_msg/msg/detail/edge__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace graph_for_task_planner_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Edge_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) graph_for_task_planner_msg::msg::Edge(_init);
}

void Edge_fini_function(void * message_memory)
{
  auto typed_message = static_cast<graph_for_task_planner_msg::msg::Edge *>(message_memory);
  typed_message->~Edge();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Edge_message_member_array[2] = {
  {
    "start_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<graph_for_task_planner_msg::msg::Point>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg::msg::Edge, start_point),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "end_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<graph_for_task_planner_msg::msg::Point>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg::msg::Edge, end_point),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Edge_message_members = {
  "graph_for_task_planner_msg::msg",  // message namespace
  "Edge",  // message name
  2,  // number of fields
  sizeof(graph_for_task_planner_msg::msg::Edge),
  Edge_message_member_array,  // message members
  Edge_init_function,  // function to initialize message memory (memory has to be allocated)
  Edge_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Edge_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Edge_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace graph_for_task_planner_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<graph_for_task_planner_msg::msg::Edge>()
{
  return &::graph_for_task_planner_msg::msg::rosidl_typesupport_introspection_cpp::Edge_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, graph_for_task_planner_msg, msg, Edge)() {
  return &::graph_for_task_planner_msg::msg::rosidl_typesupport_introspection_cpp::Edge_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
