// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/imu_raw_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_IMURawData_imu4
{
public:
  explicit Init_IMURawData_imu4(::wearable_robot_interfaces::msg::IMURawData & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::IMURawData imu4(::wearable_robot_interfaces::msg::IMURawData::_imu4_type arg)
  {
    msg_.imu4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

class Init_IMURawData_imu3
{
public:
  explicit Init_IMURawData_imu3(::wearable_robot_interfaces::msg::IMURawData & msg)
  : msg_(msg)
  {}
  Init_IMURawData_imu4 imu3(::wearable_robot_interfaces::msg::IMURawData::_imu3_type arg)
  {
    msg_.imu3 = std::move(arg);
    return Init_IMURawData_imu4(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

class Init_IMURawData_imu2
{
public:
  explicit Init_IMURawData_imu2(::wearable_robot_interfaces::msg::IMURawData & msg)
  : msg_(msg)
  {}
  Init_IMURawData_imu3 imu2(::wearable_robot_interfaces::msg::IMURawData::_imu2_type arg)
  {
    msg_.imu2 = std::move(arg);
    return Init_IMURawData_imu3(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

class Init_IMURawData_imu1
{
public:
  explicit Init_IMURawData_imu1(::wearable_robot_interfaces::msg::IMURawData & msg)
  : msg_(msg)
  {}
  Init_IMURawData_imu2 imu1(::wearable_robot_interfaces::msg::IMURawData::_imu1_type arg)
  {
    msg_.imu1 = std::move(arg);
    return Init_IMURawData_imu2(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

class Init_IMURawData_can_id
{
public:
  explicit Init_IMURawData_can_id(::wearable_robot_interfaces::msg::IMURawData & msg)
  : msg_(msg)
  {}
  Init_IMURawData_imu1 can_id(::wearable_robot_interfaces::msg::IMURawData::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_IMURawData_imu1(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

class Init_IMURawData_header
{
public:
  Init_IMURawData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMURawData_can_id header(::wearable_robot_interfaces::msg::IMURawData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_IMURawData_can_id(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMURawData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::IMURawData>()
{
  return wearable_robot_interfaces::msg::builder::Init_IMURawData_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__BUILDER_HPP_
