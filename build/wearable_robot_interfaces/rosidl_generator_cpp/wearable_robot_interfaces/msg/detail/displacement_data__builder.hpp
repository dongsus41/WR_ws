// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/DisplacementData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_DATA__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_DATA__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/displacement_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_DisplacementData_displacement
{
public:
  explicit Init_DisplacementData_displacement(::wearable_robot_interfaces::msg::DisplacementData & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::DisplacementData displacement(::wearable_robot_interfaces::msg::DisplacementData::_displacement_type arg)
  {
    msg_.displacement = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::DisplacementData msg_;
};

class Init_DisplacementData_header
{
public:
  Init_DisplacementData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DisplacementData_displacement header(::wearable_robot_interfaces::msg::DisplacementData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DisplacementData_displacement(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::DisplacementData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::DisplacementData>()
{
  return wearable_robot_interfaces::msg::builder::Init_DisplacementData_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_DATA__BUILDER_HPP_
