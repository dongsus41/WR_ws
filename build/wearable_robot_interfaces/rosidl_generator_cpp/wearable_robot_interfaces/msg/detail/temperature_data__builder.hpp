// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/TemperatureData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/temperature_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TemperatureData_temperature
{
public:
  explicit Init_TemperatureData_temperature(::wearable_robot_interfaces::msg::TemperatureData & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::TemperatureData temperature(::wearable_robot_interfaces::msg::TemperatureData::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::TemperatureData msg_;
};

class Init_TemperatureData_header
{
public:
  Init_TemperatureData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TemperatureData_temperature header(::wearable_robot_interfaces::msg::TemperatureData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TemperatureData_temperature(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::TemperatureData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::TemperatureData>()
{
  return wearable_robot_interfaces::msg::builder::Init_TemperatureData_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__TEMPERATURE_DATA__BUILDER_HPP_
