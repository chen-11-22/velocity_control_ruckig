// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tm_msgs:srv/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
#define TM_MSGS__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_

#include "tm_msgs/srv/detail/set_velocity__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace tm_msgs
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Request_velocity
{
public:
  explicit Init_SetVelocity_Request_velocity(::tm_msgs::srv::SetVelocity_Request & msg)
  : msg_(msg)
  {}
  ::tm_msgs::srv::SetVelocity_Request velocity(::tm_msgs::srv::SetVelocity_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_msgs::srv::SetVelocity_Request msg_;
};

class Init_SetVelocity_Request_motion_type
{
public:
  Init_SetVelocity_Request_motion_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Request_velocity motion_type(::tm_msgs::srv::SetVelocity_Request::_motion_type_type arg)
  {
    msg_.motion_type = std::move(arg);
    return Init_SetVelocity_Request_velocity(msg_);
  }

private:
  ::tm_msgs::srv::SetVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_msgs::srv::SetVelocity_Request>()
{
  return tm_msgs::srv::builder::Init_SetVelocity_Request_motion_type();
}

}  // namespace tm_msgs


namespace tm_msgs
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Response_ok
{
public:
  Init_SetVelocity_Response_ok()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tm_msgs::srv::SetVelocity_Response ok(::tm_msgs::srv::SetVelocity_Response::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_msgs::srv::SetVelocity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_msgs::srv::SetVelocity_Response>()
{
  return tm_msgs::srv::builder::Init_SetVelocity_Response_ok();
}

}  // namespace tm_msgs

#endif  // TM_MSGS__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
