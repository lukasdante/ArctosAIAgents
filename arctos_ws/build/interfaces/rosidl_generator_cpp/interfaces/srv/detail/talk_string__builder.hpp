// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/TalkString.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TALK_STRING__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__TALK_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/talk_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_TalkString_Request_text
{
public:
  Init_TalkString_Request_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::TalkString_Request text(::interfaces::srv::TalkString_Request::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::TalkString_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::TalkString_Request>()
{
  return interfaces::srv::builder::Init_TalkString_Request_text();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_TalkString_Response_success
{
public:
  Init_TalkString_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::TalkString_Response success(::interfaces::srv::TalkString_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::TalkString_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::TalkString_Response>()
{
  return interfaces::srv::builder::Init_TalkString_Response_success();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__TALK_STRING__BUILDER_HPP_
