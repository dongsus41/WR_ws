// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/ActuatorCommand.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/actuator_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ActuatorCommand_pwm
{
public:
  explicit Init_ActuatorCommand_pwm(::wearable_robot_interfaces::msg::ActuatorCommand & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::ActuatorCommand pwm(::wearable_robot_interfaces::msg::ActuatorCommand::_pwm_type arg)
  {
    msg_.pwm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::ActuatorCommand msg_;
};

class Init_ActuatorCommand_header
{
public:
  Init_ActuatorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActuatorCommand_pwm header(::wearable_robot_interfaces::msg::ActuatorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ActuatorCommand_pwm(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::ActuatorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::ActuatorCommand>()
{
  return wearable_robot_interfaces::msg::builder::Init_ActuatorCommand_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__ACTUATOR_COMMAND__BUILDER_HPP_
