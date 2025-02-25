// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_socketcan_msgs:msg/FdFrame.idl
// generated code does not contain a copyright notice

#ifndef ROS2_SOCKETCAN_MSGS__MSG__DETAIL__FD_FRAME__TRAITS_HPP_
#define ROS2_SOCKETCAN_MSGS__MSG__DETAIL__FD_FRAME__TRAITS_HPP_

#include "ros2_socketcan_msgs/msg/detail/fd_frame__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_socketcan_msgs::msg::FdFrame>()
{
  return "ros2_socketcan_msgs::msg::FdFrame";
}

template<>
inline const char * name<ros2_socketcan_msgs::msg::FdFrame>()
{
  return "ros2_socketcan_msgs/msg/FdFrame";
}

template<>
struct has_fixed_size<ros2_socketcan_msgs::msg::FdFrame>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_socketcan_msgs::msg::FdFrame>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ros2_socketcan_msgs::msg::FdFrame>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_SOCKETCAN_MSGS__MSG__DETAIL__FD_FRAME__TRAITS_HPP_
