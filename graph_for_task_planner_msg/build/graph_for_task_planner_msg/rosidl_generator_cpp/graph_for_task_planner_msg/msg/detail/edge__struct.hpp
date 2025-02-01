// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_HPP_
#define GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "graph_for_task_planner_msg/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__graph_for_task_planner_msg__msg__Edge __attribute__((deprecated))
#else
# define DEPRECATED__graph_for_task_planner_msg__msg__Edge __declspec(deprecated)
#endif

namespace graph_for_task_planner_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Edge_
{
  using Type = Edge_<ContainerAllocator>;

  explicit Edge_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_point(_init),
    end_point(_init)
  {
    (void)_init;
  }

  explicit Edge_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_point(_alloc, _init),
    end_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_point_type =
    graph_for_task_planner_msg::msg::Point_<ContainerAllocator>;
  _start_point_type start_point;
  using _end_point_type =
    graph_for_task_planner_msg::msg::Point_<ContainerAllocator>;
  _end_point_type end_point;

  // setters for named parameter idiom
  Type & set__start_point(
    const graph_for_task_planner_msg::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start_point = _arg;
    return *this;
  }
  Type & set__end_point(
    const graph_for_task_planner_msg::msg::Point_<ContainerAllocator> & _arg)
  {
    this->end_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> *;
  using ConstRawPtr =
    const graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graph_for_task_planner_msg__msg__Edge
    std::shared_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graph_for_task_planner_msg__msg__Edge
    std::shared_ptr<graph_for_task_planner_msg::msg::Edge_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Edge_ & other) const
  {
    if (this->start_point != other.start_point) {
      return false;
    }
    if (this->end_point != other.end_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const Edge_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Edge_

// alias to use template instance with default allocator
using Edge =
  graph_for_task_planner_msg::msg::Edge_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace graph_for_task_planner_msg

#endif  // GRAPH_FOR_TASK_PLANNER_MSG__MSG__DETAIL__EDGE__STRUCT_HPP_
