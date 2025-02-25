// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__TRAITS_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__TRAITS_HPP_

#include "wearable_robot_interfaces/msg/detail/imu_type__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<wearable_robot_interfaces::msg::IMUType>()
{
  return "wearable_robot_interfaces::msg::IMUType";
}

template<>
inline const char * name<wearable_robot_interfaces::msg::IMUType>()
{
  return "wearable_robot_interfaces/msg/IMUType";
}

template<>
struct has_fixed_size<wearable_robot_interfaces::msg::IMUType>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wearable_robot_interfaces::msg::IMUType>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wearable_robot_interfaces::msg::IMUType>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__TRAITS_HPP_
