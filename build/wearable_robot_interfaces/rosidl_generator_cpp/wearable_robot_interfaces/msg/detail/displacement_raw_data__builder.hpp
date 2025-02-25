// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/DisplacementRawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/displacement_raw_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_DisplacementRawData_displacement_raw
{
public:
  explicit Init_DisplacementRawData_displacement_raw(::wearable_robot_interfaces::msg::DisplacementRawData & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::DisplacementRawData displacement_raw(::wearable_robot_interfaces::msg::DisplacementRawData::_displacement_raw_type arg)
  {
    msg_.displacement_raw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::DisplacementRawData msg_;
};

class Init_DisplacementRawData_header
{
public:
  Init_DisplacementRawData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DisplacementRawData_displacement_raw header(::wearable_robot_interfaces::msg::DisplacementRawData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DisplacementRawData_displacement_raw(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::DisplacementRawData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::DisplacementRawData>()
{
  return wearable_robot_interfaces::msg::builder::Init_DisplacementRawData_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__DISPLACEMENT_RAW_DATA__BUILDER_HPP_
