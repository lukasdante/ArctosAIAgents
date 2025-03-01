// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_NodeStatus_node_list
{
public:
  explicit Init_NodeStatus_node_list(::interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::NodeStatus node_list(::interfaces::msg::NodeStatus::_node_list_type arg)
  {
    msg_.node_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_node_status
{
public:
  Init_NodeStatus_node_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NodeStatus_node_list node_status(::interfaces::msg::NodeStatus::_node_status_type arg)
  {
    msg_.node_status = std::move(arg);
    return Init_NodeStatus_node_list(msg_);
  }

private:
  ::interfaces::msg::NodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::NodeStatus>()
{
  return interfaces::msg::builder::Init_NodeStatus_node_status();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
