// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/FanCommand.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/fan_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_FanCommand_fan
{
public:
  explicit Init_FanCommand_fan(::wearable_robot_interfaces::msg::FanCommand & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::FanCommand fan(::wearable_robot_interfaces::msg::FanCommand::_fan_type arg)
  {
    msg_.fan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::FanCommand msg_;
};

class Init_FanCommand_header
{
public:
  Init_FanCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FanCommand_fan header(::wearable_robot_interfaces::msg::FanCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FanCommand_fan(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::FanCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::FanCommand>()
{
  return wearable_robot_interfaces::msg::builder::Init_FanCommand_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__BUILDER_HPP_
