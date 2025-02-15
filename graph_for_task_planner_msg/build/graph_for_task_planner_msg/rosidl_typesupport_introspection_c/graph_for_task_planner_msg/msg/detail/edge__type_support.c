// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "graph_for_task_planner_msg/msg/detail/edge__rosidl_typesupport_introspection_c.h"
#include "graph_for_task_planner_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "graph_for_task_planner_msg/msg/detail/edge__functions.h"
#include "graph_for_task_planner_msg/msg/detail/edge__struct.h"


// Include directives for member types
// Member `start_point`
// Member `end_point`
#include "graph_for_task_planner_msg/msg/point.h"
// Member `start_point`
// Member `end_point`
#include "graph_for_task_planner_msg/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  graph_for_task_planner_msg__msg__Edge__init(message_memory);
}

void graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_fini_function(void * message_memory)
{
  graph_for_task_planner_msg__msg__Edge__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_member_array[2] = {
  {
    "start_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg__msg__Edge, start_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg__msg__Edge, end_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_members = {
  "graph_for_task_planner_msg__msg",  // message namespace
  "Edge",  // message name
  2,  // number of fields
  sizeof(graph_for_task_planner_msg__msg__Edge),
  graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_member_array,  // message members
  graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_init_function,  // function to initialize message memory (memory has to be allocated)
  graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_type_support_handle = {
  0,
  &graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_graph_for_task_planner_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Edge)() {
  graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Point)();
  graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Point)();
  if (!graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_type_support_handle.typesupport_identifier) {
    graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &graph_for_task_planner_msg__msg__Edge__rosidl_typesupport_introspection_c__Edge_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
