// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from graph_for_task_planner_msg:msg/Graph.idl
// generated code does not contain a copyright notice
#include "graph_for_task_planner_msg/msg/detail/graph__rosidl_typesupport_fastrtps_cpp.hpp"
#include "graph_for_task_planner_msg/msg/detail/graph__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace graph_for_task_planner_msg
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const graph_for_task_planner_msg::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  graph_for_task_planner_msg::msg::Point &);
size_t get_serialized_size(
  const graph_for_task_planner_msg::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace graph_for_task_planner_msg

namespace graph_for_task_planner_msg
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const graph_for_task_planner_msg::msg::Edge &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  graph_for_task_planner_msg::msg::Edge &);
size_t get_serialized_size(
  const graph_for_task_planner_msg::msg::Edge &,
  size_t current_alignment);
size_t
max_serialized_size_Edge(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace graph_for_task_planner_msg


namespace graph_for_task_planner_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_graph_for_task_planner_msg
cdr_serialize(
  const graph_for_task_planner_msg::msg::Graph & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vertices
  {
    size_t size = ros_message.vertices.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.vertices[i],
        cdr);
    }
  }
  // Member: edges
  {
    size_t size = ros_message.edges.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.edges[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_graph_for_task_planner_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  graph_for_task_planner_msg::msg::Graph & ros_message)
{
  // Member: vertices
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.vertices.resize(size);
    for (size_t i = 0; i < size; i++) {
      graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.vertices[i]);
    }
  }

  // Member: edges
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.edges.resize(size);
    for (size_t i = 0; i < size; i++) {
      graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.edges[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_graph_for_task_planner_msg
get_serialized_size(
  const graph_for_task_planner_msg::msg::Graph & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vertices
  {
    size_t array_size = ros_message.vertices.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.vertices[index], current_alignment);
    }
  }
  // Member: edges
  {
    size_t array_size = ros_message.edges.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.edges[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_graph_for_task_planner_msg
max_serialized_size_Graph(
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


  // Member: vertices
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: edges
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::max_serialized_size_Edge(
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
    using DataType = graph_for_task_planner_msg::msg::Graph;
    is_plain =
      (
      offsetof(DataType, edges) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Graph__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const graph_for_task_planner_msg::msg::Graph *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Graph__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<graph_for_task_planner_msg::msg::Graph *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Graph__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const graph_for_task_planner_msg::msg::Graph *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Graph__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Graph(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Graph__callbacks = {
  "graph_for_task_planner_msg::msg",
  "Graph",
  _Graph__cdr_serialize,
  _Graph__cdr_deserialize,
  _Graph__get_serialized_size,
  _Graph__max_serialized_size
};

static rosidl_message_type_support_t _Graph__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Graph__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace graph_for_task_planner_msg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_graph_for_task_planner_msg
const rosidl_message_type_support_t *
get_message_type_support_handle<graph_for_task_planner_msg::msg::Graph>()
{
  return &graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::_Graph__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, graph_for_task_planner_msg, msg, Graph)() {
  return &graph_for_task_planner_msg::msg::typesupport_fastrtps_cpp::_Graph__handle;
}

#ifdef __cplusplus
}
#endif
