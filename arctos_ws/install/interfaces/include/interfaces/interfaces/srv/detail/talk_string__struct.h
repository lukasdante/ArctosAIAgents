// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/TalkString.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TALK_STRING__STRUCT_H_
#define INTERFACES__SRV__DETAIL__TALK_STRING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TalkString in the package interfaces.
typedef struct interfaces__srv__TalkString_Request
{
  rosidl_runtime_c__String text;
} interfaces__srv__TalkString_Request;

// Struct for a sequence of interfaces__srv__TalkString_Request.
typedef struct interfaces__srv__TalkString_Request__Sequence
{
  interfaces__srv__TalkString_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__TalkString_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TalkString in the package interfaces.
typedef struct interfaces__srv__TalkString_Response
{
  bool success;
} interfaces__srv__TalkString_Response;

// Struct for a sequence of interfaces__srv__TalkString_Response.
typedef struct interfaces__srv__TalkString_Response__Sequence
{
  interfaces__srv__TalkString_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__TalkString_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__TALK_STRING__STRUCT_H_
