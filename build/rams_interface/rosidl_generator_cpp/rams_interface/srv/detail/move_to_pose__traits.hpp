// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rams_interface:srv/MoveToPose.idl
// generated code does not contain a copyright notice

#ifndef RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__TRAITS_HPP_
#define RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rams_interface/srv/detail/move_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace rams_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveToPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    to_flow_style_yaml(msg.target, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target:\n";
    to_block_style_yaml(msg.target, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPose_Request & msg, bool use_flow_style = false)
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

}  // namespace rams_interface

namespace rosidl_generator_traits
{

[[deprecated("use rams_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rams_interface::srv::MoveToPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rams_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rams_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const rams_interface::srv::MoveToPose_Request & msg)
{
  return rams_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rams_interface::srv::MoveToPose_Request>()
{
  return "rams_interface::srv::MoveToPose_Request";
}

template<>
inline const char * name<rams_interface::srv::MoveToPose_Request>()
{
  return "rams_interface/srv/MoveToPose_Request";
}

template<>
struct has_fixed_size<rams_interface::srv::MoveToPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<rams_interface::srv::MoveToPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<rams_interface::srv::MoveToPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rams_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveToPose_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToPose_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToPose_Response & msg, bool use_flow_style = false)
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

}  // namespace rams_interface

namespace rosidl_generator_traits
{

[[deprecated("use rams_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rams_interface::srv::MoveToPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rams_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rams_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const rams_interface::srv::MoveToPose_Response & msg)
{
  return rams_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rams_interface::srv::MoveToPose_Response>()
{
  return "rams_interface::srv::MoveToPose_Response";
}

template<>
inline const char * name<rams_interface::srv::MoveToPose_Response>()
{
  return "rams_interface/srv/MoveToPose_Response";
}

template<>
struct has_fixed_size<rams_interface::srv::MoveToPose_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rams_interface::srv::MoveToPose_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rams_interface::srv::MoveToPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rams_interface::srv::MoveToPose>()
{
  return "rams_interface::srv::MoveToPose";
}

template<>
inline const char * name<rams_interface::srv::MoveToPose>()
{
  return "rams_interface/srv/MoveToPose";
}

template<>
struct has_fixed_size<rams_interface::srv::MoveToPose>
  : std::integral_constant<
    bool,
    has_fixed_size<rams_interface::srv::MoveToPose_Request>::value &&
    has_fixed_size<rams_interface::srv::MoveToPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<rams_interface::srv::MoveToPose>
  : std::integral_constant<
    bool,
    has_bounded_size<rams_interface::srv::MoveToPose_Request>::value &&
    has_bounded_size<rams_interface::srv::MoveToPose_Response>::value
  >
{
};

template<>
struct is_service<rams_interface::srv::MoveToPose>
  : std::true_type
{
};

template<>
struct is_service_request<rams_interface::srv::MoveToPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rams_interface::srv::MoveToPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__TRAITS_HPP_
