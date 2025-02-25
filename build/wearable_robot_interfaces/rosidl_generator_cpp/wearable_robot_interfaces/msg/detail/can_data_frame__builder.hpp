// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/CANDataFrame.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/can_data_frame__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CANDataFrame_data
{
public:
  explicit Init_CANDataFrame_data(::wearable_robot_interfaces::msg::CANDataFrame & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::CANDataFrame data(::wearable_robot_interfaces::msg::CANDataFrame::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::CANDataFrame msg_;
};

class Init_CANDataFrame_header
{
public:
  Init_CANDataFrame_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CANDataFrame_data header(::wearable_robot_interfaces::msg::CANDataFrame::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CANDataFrame_data(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::CANDataFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::CANDataFrame>()
{
  return wearable_robot_interfaces::msg::builder::Init_CANDataFrame_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__BUILDER_HPP_
