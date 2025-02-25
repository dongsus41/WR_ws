// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/CANData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/can_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CANData_data
{
public:
  explicit Init_CANData_data(::wearable_robot_interfaces::msg::CANData & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::CANData data(::wearable_robot_interfaces::msg::CANData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::CANData msg_;
};

class Init_CANData_can_id
{
public:
  explicit Init_CANData_can_id(::wearable_robot_interfaces::msg::CANData & msg)
  : msg_(msg)
  {}
  Init_CANData_data can_id(::wearable_robot_interfaces::msg::CANData::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_CANData_data(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::CANData msg_;
};

class Init_CANData_header
{
public:
  Init_CANData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CANData_can_id header(::wearable_robot_interfaces::msg::CANData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CANData_can_id(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::CANData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::CANData>()
{
  return wearable_robot_interfaces::msg::builder::Init_CANData_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA__BUILDER_HPP_
