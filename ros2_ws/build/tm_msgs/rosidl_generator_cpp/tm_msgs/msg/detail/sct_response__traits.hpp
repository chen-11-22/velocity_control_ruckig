// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:msg/SctResponse.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_
#define TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_

#include "tm_msgs/msg/detail/sct_response__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tm_msgs::msg::SctResponse>()
{
  return "tm_msgs::msg::SctResponse";
}

template<>
inline const char * name<tm_msgs::msg::SctResponse>()
{
  return "tm_msgs/msg/SctResponse";
}

template<>
struct has_fixed_size<tm_msgs::msg::SctResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::msg::SctResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::msg::SctResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_
