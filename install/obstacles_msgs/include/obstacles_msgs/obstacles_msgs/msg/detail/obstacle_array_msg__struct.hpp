// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from obstacles_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_
#define OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'obstacles'
#include "obstacles_msgs/msg/detail/obstacle_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__obstacles_msgs__msg__ObstacleArrayMsg __attribute__((deprecated))
#else
# define DEPRECATED__obstacles_msgs__msg__ObstacleArrayMsg __declspec(deprecated)
#endif

namespace obstacles_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleArrayMsg_
{
  using Type = ObstacleArrayMsg_<ContainerAllocator>;

  explicit ObstacleArrayMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObstacleArrayMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _obstacles_type =
    std::vector<obstacles_msgs::msg::ObstacleMsg_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<obstacles_msgs::msg::ObstacleMsg_<ContainerAllocator>>>;
  _obstacles_type obstacles;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__obstacles(
    const std::vector<obstacles_msgs::msg::ObstacleMsg_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<obstacles_msgs::msg::ObstacleMsg_<ContainerAllocator>>> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__obstacles_msgs__msg__ObstacleArrayMsg
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__obstacles_msgs__msg__ObstacleArrayMsg
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleArrayMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->obstacles != other.obstacles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleArrayMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleArrayMsg_

// alias to use template instance with default allocator
using ObstacleArrayMsg =
  obstacles_msgs::msg::ObstacleArrayMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace obstacles_msgs

#endif  // OBSTACLES_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_
