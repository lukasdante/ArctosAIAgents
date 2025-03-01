// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NodeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: node_status
  {
    out << "node_status: ";
    rosidl_generator_traits::value_to_yaml(msg.node_status, out);
    out << ", ";
  }

  // member: node_list
  {
    if (msg.node_list.size() == 0) {
      out << "node_list: []";
    } else {
      out << "node_list: [";
      size_t pending_items = msg.node_list.size();
      for (auto item : msg.node_list) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: node_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_status: ";
    rosidl_generator_traits::value_to_yaml(msg.node_status, out);
    out << "\n";
  }

  // member: node_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.node_list.size() == 0) {
      out << "node_list: []\n";
    } else {
      out << "node_list:\n";
      for (auto item : msg.node_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NodeStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::NodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::NodeStatus & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::NodeStatus>()
{
  return "interfaces::msg::NodeStatus";
}

template<>
inline const char * name<interfaces::msg::NodeStatus>()
{
  return "interfaces/msg/NodeStatus";
}

template<>
struct has_fixed_size<interfaces::msg::NodeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::NodeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::NodeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
