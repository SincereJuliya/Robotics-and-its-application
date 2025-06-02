// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from graph_for_task_planner_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "graph_for_task_planner_msg/msg/detail/graph__rosidl_typesupport_introspection_c.h"
#include "graph_for_task_planner_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "graph_for_task_planner_msg/msg/detail/graph__functions.h"
#include "graph_for_task_planner_msg/msg/detail/graph__struct.h"


// Include directives for member types
// Member `vertices`
#include "graph_for_task_planner_msg/msg/point.h"
// Member `vertices`
#include "graph_for_task_planner_msg/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `edges`
#include "graph_for_task_planner_msg/msg/edge.h"
// Member `edges`
#include "graph_for_task_planner_msg/msg/detail/edge__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  graph_for_task_planner_msg__msg__Graph__init(message_memory);
}

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function(void * message_memory)
{
  graph_for_task_planner_msg__msg__Graph__fini(message_memory);
}

size_t graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__vertices(
  const void * untyped_member)
{
  const graph_for_task_planner_msg__msg__Point__Sequence * member =
    (const graph_for_task_planner_msg__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__vertices(
  const void * untyped_member, size_t index)
{
  const graph_for_task_planner_msg__msg__Point__Sequence * member =
    (const graph_for_task_planner_msg__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__vertices(
  void * untyped_member, size_t index)
{
  graph_for_task_planner_msg__msg__Point__Sequence * member =
    (graph_for_task_planner_msg__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__vertices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const graph_for_task_planner_msg__msg__Point * item =
    ((const graph_for_task_planner_msg__msg__Point *)
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__vertices(untyped_member, index));
  graph_for_task_planner_msg__msg__Point * value =
    (graph_for_task_planner_msg__msg__Point *)(untyped_value);
  *value = *item;
}

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__vertices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  graph_for_task_planner_msg__msg__Point * item =
    ((graph_for_task_planner_msg__msg__Point *)
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__vertices(untyped_member, index));
  const graph_for_task_planner_msg__msg__Point * value =
    (const graph_for_task_planner_msg__msg__Point *)(untyped_value);
  *item = *value;
}

bool graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__vertices(
  void * untyped_member, size_t size)
{
  graph_for_task_planner_msg__msg__Point__Sequence * member =
    (graph_for_task_planner_msg__msg__Point__Sequence *)(untyped_member);
  graph_for_task_planner_msg__msg__Point__Sequence__fini(member);
  return graph_for_task_planner_msg__msg__Point__Sequence__init(member, size);
}

size_t graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__edges(
  const void * untyped_member)
{
  const graph_for_task_planner_msg__msg__Edge__Sequence * member =
    (const graph_for_task_planner_msg__msg__Edge__Sequence *)(untyped_member);
  return member->size;
}

const void * graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges(
  const void * untyped_member, size_t index)
{
  const graph_for_task_planner_msg__msg__Edge__Sequence * member =
    (const graph_for_task_planner_msg__msg__Edge__Sequence *)(untyped_member);
  return &member->data[index];
}

void * graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges(
  void * untyped_member, size_t index)
{
  graph_for_task_planner_msg__msg__Edge__Sequence * member =
    (graph_for_task_planner_msg__msg__Edge__Sequence *)(untyped_member);
  return &member->data[index];
}

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__edges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const graph_for_task_planner_msg__msg__Edge * item =
    ((const graph_for_task_planner_msg__msg__Edge *)
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges(untyped_member, index));
  graph_for_task_planner_msg__msg__Edge * value =
    (graph_for_task_planner_msg__msg__Edge *)(untyped_value);
  *value = *item;
}

void graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__edges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  graph_for_task_planner_msg__msg__Edge * item =
    ((graph_for_task_planner_msg__msg__Edge *)
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges(untyped_member, index));
  const graph_for_task_planner_msg__msg__Edge * value =
    (const graph_for_task_planner_msg__msg__Edge *)(untyped_value);
  *item = *value;
}

bool graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__edges(
  void * untyped_member, size_t size)
{
  graph_for_task_planner_msg__msg__Edge__Sequence * member =
    (graph_for_task_planner_msg__msg__Edge__Sequence *)(untyped_member);
  graph_for_task_planner_msg__msg__Edge__Sequence__fini(member);
  return graph_for_task_planner_msg__msg__Edge__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[2] = {
  {
    "vertices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg__msg__Graph, vertices),  // bytes offset in struct
    NULL,  // default value
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__vertices,  // size() function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__vertices,  // get_const(index) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__vertices,  // get(index) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__vertices,  // fetch(index, &value) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__vertices,  // assign(index, value) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__vertices  // resize(index) function pointer
  },
  {
    "edges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_for_task_planner_msg__msg__Graph, edges),  // bytes offset in struct
    NULL,  // default value
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__edges,  // size() function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges,  // get_const(index) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges,  // get(index) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__edges,  // fetch(index, &value) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__edges,  // assign(index, value) function pointer
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__edges  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members = {
  "graph_for_task_planner_msg__msg",  // message namespace
  "Graph",  // message name
  2,  // number of fields
  sizeof(graph_for_task_planner_msg__msg__Graph),
  graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array,  // message members
  graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function,  // function to initialize message memory (memory has to be allocated)
  graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle = {
  0,
  &graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_graph_for_task_planner_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Graph)() {
  graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Point)();
  graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_for_task_planner_msg, msg, Edge)();
  if (!graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier) {
    graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &graph_for_task_planner_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
