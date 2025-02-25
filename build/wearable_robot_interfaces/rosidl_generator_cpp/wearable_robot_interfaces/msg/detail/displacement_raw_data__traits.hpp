// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wearable_robot_interfaces:msg/DisplacementRawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__TRAITS_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__TRAITS_HPP_

#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<wearable_robot_interfaces::msg::DisplacementRawData>()
{
  return "wearable_robot_interfaces::msg::DisplacementRawData";
}

template<>
inline const char * name<wearable_robot_interfaces::msg::DisplacementRawData>()
{
  return "wearable_robot_interfaces/msg/DisplacementRawData";
}

template<>
struct has_fixed_size<wearable_robot_interfaces::msg::DisplacementRawData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wearable_robot_interfaces::msg::DisplacementRawData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wearable_robot_interfaces::msg::DisplacementRawData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__TRAITS_HPP_
