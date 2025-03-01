// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__NodeStatus __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__NodeStatus __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NodeStatus_
{
  using Type = NodeStatus_<ContainerAllocator>;

  explicit NodeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_status = false;
    }
  }

  explicit NodeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_status = false;
    }
  }

  // field types and members
  using _node_status_type =
    bool;
  _node_status_type node_status;
  using _node_list_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _node_list_type node_list;

  // setters for named parameter idiom
  Type & set__node_status(
    const bool & _arg)
  {
    this->node_status = _arg;
    return *this;
  }
  Type & set__node_list(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->node_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::NodeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::NodeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::NodeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::NodeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::NodeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::NodeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::NodeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::NodeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::NodeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::NodeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__NodeStatus
    std::shared_ptr<interfaces::msg::NodeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__NodeStatus
    std::shared_ptr<interfaces::msg::NodeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NodeStatus_ & other) const
  {
    if (this->node_status != other.node_status) {
      return false;
    }
    if (this->node_list != other.node_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const NodeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NodeStatus_

// alias to use template instance with default allocator
using NodeStatus =
  interfaces::msg::NodeStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
