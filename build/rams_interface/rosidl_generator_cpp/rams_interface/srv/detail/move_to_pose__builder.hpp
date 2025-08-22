// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rams_interface:srv/MoveToPose.idl
// generated code does not contain a copyright notice

#ifndef RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__BUILDER_HPP_
#define RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rams_interface/srv/detail/move_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rams_interface
{

namespace srv
{

namespace builder
{

class Init_MoveToPose_Request_target
{
public:
  Init_MoveToPose_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rams_interface::srv::MoveToPose_Request target(::rams_interface::srv::MoveToPose_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rams_interface::srv::MoveToPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rams_interface::srv::MoveToPose_Request>()
{
  return rams_interface::srv::builder::Init_MoveToPose_Request_target();
}

}  // namespace rams_interface


namespace rams_interface
{

namespace srv
{

namespace builder
{

class Init_MoveToPose_Response_message
{
public:
  explicit Init_MoveToPose_Response_message(::rams_interface::srv::MoveToPose_Response & msg)
  : msg_(msg)
  {}
  ::rams_interface::srv::MoveToPose_Response message(::rams_interface::srv::MoveToPose_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rams_interface::srv::MoveToPose_Response msg_;
};

class Init_MoveToPose_Response_success
{
public:
  Init_MoveToPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPose_Response_message success(::rams_interface::srv::MoveToPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveToPose_Response_message(msg_);
  }

private:
  ::rams_interface::srv::MoveToPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rams_interface::srv::MoveToPose_Response>()
{
  return rams_interface::srv::builder::Init_MoveToPose_Response_success();
}

}  // namespace rams_interface

#endif  // RAMS_INTERFACE__SRV__DETAIL__MOVE_TO_POSE__BUILDER_HPP_
