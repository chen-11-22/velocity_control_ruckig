// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tm_msgs:srv/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__SET_VELOCITY__STRUCT_H_
#define TM_MSGS__SRV__DETAIL__SET_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'VEL_J'.
enum
{
  tm_msgs__srv__SetVelocity_Request__VEL_J = 1
};

/// Constant 'VEL_T'.
enum
{
  tm_msgs__srv__SetVelocity_Request__VEL_T = 2
};

// Include directives for member types
// Member 'velocity'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/SetVelocity in the package tm_msgs.
typedef struct tm_msgs__srv__SetVelocity_Request
{
  int8_t motion_type;
  rosidl_runtime_c__double__Sequence velocity;
} tm_msgs__srv__SetVelocity_Request;

// Struct for a sequence of tm_msgs__srv__SetVelocity_Request.
typedef struct tm_msgs__srv__SetVelocity_Request__Sequence
{
  tm_msgs__srv__SetVelocity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__SetVelocity_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetVelocity in the package tm_msgs.
typedef struct tm_msgs__srv__SetVelocity_Response
{
  bool ok;
} tm_msgs__srv__SetVelocity_Response;

// Struct for a sequence of tm_msgs__srv__SetVelocity_Response.
typedef struct tm_msgs__srv__SetVelocity_Response__Sequence
{
  tm_msgs__srv__SetVelocity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_msgs__srv__SetVelocity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TM_MSGS__SRV__DETAIL__SET_VELOCITY__STRUCT_H_
