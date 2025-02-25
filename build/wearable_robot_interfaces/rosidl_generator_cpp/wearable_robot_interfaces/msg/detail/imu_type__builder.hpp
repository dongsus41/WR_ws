// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/IMUType.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/imu_type__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_IMUType_yaw
{
public:
  explicit Init_IMUType_yaw(::wearable_robot_interfaces::msg::IMUType & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::IMUType yaw(::wearable_robot_interfaces::msg::IMUType::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMUType msg_;
};

class Init_IMUType_pitch
{
public:
  explicit Init_IMUType_pitch(::wearable_robot_interfaces::msg::IMUType & msg)
  : msg_(msg)
  {}
  Init_IMUType_yaw pitch(::wearable_robot_interfaces::msg::IMUType::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_IMUType_yaw(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMUType msg_;
};

class Init_IMUType_roll
{
public:
  Init_IMUType_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMUType_pitch roll(::wearable_robot_interfaces::msg::IMUType::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_IMUType_pitch(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::IMUType msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::IMUType>()
{
  return wearable_robot_interfaces::msg::builder::Init_IMUType_roll();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_TYPE__BUILDER_HPP_
