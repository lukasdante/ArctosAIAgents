// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__RESPONSE__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Response_parameters
{
public:
  explicit Init_Response_parameters(::interfaces::msg::Response & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Response parameters(::interfaces::msg::Response::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Response msg_;
};

class Init_Response_response
{
public:
  Init_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Response_parameters response(::interfaces::msg::Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_Response_parameters(msg_);
  }

private:
  ::interfaces::msg::Response msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Response>()
{
  return interfaces::msg::builder::Init_Response_response();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__RESPONSE__BUILDER_HPP_
