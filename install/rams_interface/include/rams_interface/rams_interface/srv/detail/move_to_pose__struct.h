// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rams_interface:srv/MoveToPose.idl
// generated code does not contain a copyright notice

#ifndef RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__STRUCT_H_
#define RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/MoveToPose in the package rams_interface.
typedef struct rams_interface__srv__MoveToPose_Request
{
  geometry_msgs__msg__PoseStamped target;
} rams_interface__srv__MoveToPose_Request;

// Struct for a sequence of rams_interface__srv__MoveToPose_Request.
typedef struct rams_interface__srv__MoveToPose_Request__Sequence
{
  rams_interface__srv__MoveToPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rams_interface__srv__MoveToPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToPose in the package rams_interface.
typedef struct rams_interface__srv__MoveToPose_Response
{
  bool success;
  rosidl_runtime_c__String message;
} rams_interface__srv__MoveToPose_Response;

// Struct for a sequence of rams_interface__srv__MoveToPose_Response.
typedef struct rams_interface__srv__MoveToPose_Response__Sequence
{
  rams_interface__srv__MoveToPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rams_interface__srv__MoveToPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__STRUCT_H_
