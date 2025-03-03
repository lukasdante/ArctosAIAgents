// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__RESPONSE__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__RESPONSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Response_
{
  using Type = Response_<ContainerAllocator>;

  explicit Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
      this->parameters = "";
    }
  }

  explicit Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc),
    parameters(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
      this->parameters = "";
    }
  }

  // field types and members
  using _response_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _response_type response;
  using _parameters_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _parameters_type parameters;

  // setters for named parameter idiom
  Type & set__response(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->response = _arg;
    return *this;
  }
  Type & set__parameters(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Response
    std::shared_ptr<interfaces::msg::Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Response
    std::shared_ptr<interfaces::msg::Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    if (this->parameters != other.parameters) {
      return false;
    }
    return true;
  }
  bool operator!=(const Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Response_

// alias to use template instance with default allocator
using Response =
  interfaces::msg::Response_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__RESPONSE__STRUCT_HPP_
