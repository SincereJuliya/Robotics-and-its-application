// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice
#include "graph_for_task_planner_msg/msg/detail/edge__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "graph_for_task_planner_msg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "graph_for_task_planner_msg/msg/detail/edge__struct.h"
#include "graph_for_task_planner_msg/msg/detail/edge__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "graph_for_task_planner_msg/msg/detail/point__functions.h"  // end_point, start_point

// forward declare type support functions
size_t get_serialized_size_graph_for_task_planner_msg__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_graph_for_task_planner_msg__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Point)();


using _Edge__ros_msg_type = graph_for_task_planner_msg__msg__Edge;

static bool _Edge__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Edge__ros_msg_type * ros_message = static_cast<const _Edge__ros_msg_type *>(untyped_ros_message);
  // Field name: start_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->start_point, cdr))
    {
      return false;
    }
  }

  // Field name: end_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->end_point, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _Edge__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Edge__ros_msg_type * ros_message = static_cast<_Edge__ros_msg_type *>(untyped_ros_message);
  // Field name: start_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->start_point))
    {
      return false;
    }
  }

  // Field name: end_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->end_point))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_graph_for_task_planner_msg
size_t get_serialized_size_graph_for_task_planner_msg__msg__Edge(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Edge__ros_msg_type * ros_message = static_cast<const _Edge__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name start_point

  current_alignment += get_serialized_size_graph_for_task_planner_msg__msg__Point(
    &(ros_message->start_point), current_alignment);
  // field.name end_point

  current_alignment += get_serialized_size_graph_for_task_planner_msg__msg__Point(
    &(ros_message->end_point), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _Edge__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_graph_for_task_planner_msg__msg__Edge(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_graph_for_task_planner_msg
size_t max_serialized_size_graph_for_task_planner_msg__msg__Edge(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: start_point
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_graph_for_task_planner_msg__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: end_point
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_graph_for_task_planner_msg__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = graph_for_task_planner_msg__msg__Edge;
    is_plain =
      (
      offsetof(DataType, end_point) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Edge__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_graph_for_task_planner_msg__msg__Edge(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Edge = {
  "graph_for_task_planner_msg::msg",
  "Edge",
  _Edge__cdr_serialize,
  _Edge__cdr_deserialize,
  _Edge__get_serialized_size,
  _Edge__max_serialized_size
};

static rosidl_message_type_support_t _Edge__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Edge,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, graph_for_task_planner_msg, msg, Edge)() {
  return &_Edge__type_support;
}

#if defined(__cplusplus)
}
#endif
