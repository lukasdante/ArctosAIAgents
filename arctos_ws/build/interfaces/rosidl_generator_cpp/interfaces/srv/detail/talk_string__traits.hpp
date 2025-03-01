// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/TalkString.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TALK_STRING__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__TALK_STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/talk_string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TalkString_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: text
  {
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TalkString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TalkString_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::TalkString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::TalkString_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::TalkString_Request>()
{
  return "interfaces::srv::TalkString_Request";
}

template<>
inline const char * name<interfaces::srv::TalkString_Request>()
{
  return "interfaces/srv/TalkString_Request";
}

template<>
struct has_fixed_size<interfaces::srv::TalkString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::TalkString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::TalkString_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const TalkString_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TalkString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TalkString_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::TalkString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::TalkString_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::TalkString_Response>()
{
  return "interfaces::srv::TalkString_Response";
}

template<>
inline const char * name<interfaces::srv::TalkString_Response>()
{
  return "interfaces/srv/TalkString_Response";
}

template<>
struct has_fixed_size<interfaces::srv::TalkString_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::TalkString_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::TalkString_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::TalkString>()
{
  return "interfaces::srv::TalkString";
}

template<>
inline const char * name<interfaces::srv::TalkString>()
{
  return "interfaces/srv/TalkString";
}

template<>
struct has_fixed_size<interfaces::srv::TalkString>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::TalkString_Request>::value &&
    has_fixed_size<interfaces::srv::TalkString_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::TalkString>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::TalkString_Request>::value &&
    has_bounded_size<interfaces::srv::TalkString_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::TalkString>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::TalkString_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::TalkString_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__TALK_STRING__TRAITS_HPP_
