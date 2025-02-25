// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wearable_robot_interfaces:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_

#include "wearable_robot_interfaces/msg/detail/joint_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace wearable_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_JointState_l_elbow_angle
{
public:
  explicit Init_JointState_l_elbow_angle(::wearable_robot_interfaces::msg::JointState & msg)
  : msg_(msg)
  {}
  ::wearable_robot_interfaces::msg::JointState l_elbow_angle(::wearable_robot_interfaces::msg::JointState::_l_elbow_angle_type arg)
  {
    msg_.l_elbow_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::JointState msg_;
};

class Init_JointState_r_elbow_angle
{
public:
  explicit Init_JointState_r_elbow_angle(::wearable_robot_interfaces::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_l_elbow_angle r_elbow_angle(::wearable_robot_interfaces::msg::JointState::_r_elbow_angle_type arg)
  {
    msg_.r_elbow_angle = std::move(arg);
    return Init_JointState_l_elbow_angle(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::JointState msg_;
};

class Init_JointState_l_shoulder_angle
{
public:
  explicit Init_JointState_l_shoulder_angle(::wearable_robot_interfaces::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_r_elbow_angle l_shoulder_angle(::wearable_robot_interfaces::msg::JointState::_l_shoulder_angle_type arg)
  {
    msg_.l_shoulder_angle = std::move(arg);
    return Init_JointState_r_elbow_angle(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::JointState msg_;
};

class Init_JointState_r_shoulder_angle
{
public:
  explicit Init_JointState_r_shoulder_angle(::wearable_robot_interfaces::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_l_shoulder_angle r_shoulder_angle(::wearable_robot_interfaces::msg::JointState::_r_shoulder_angle_type arg)
  {
    msg_.r_shoulder_angle = std::move(arg);
    return Init_JointState_l_shoulder_angle(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::JointState msg_;
};

class Init_JointState_header
{
public:
  Init_JointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointState_r_shoulder_angle header(::wearable_robot_interfaces::msg::JointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointState_r_shoulder_angle(msg_);
  }

private:
  ::wearable_robot_interfaces::msg::JointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wearable_robot_interfaces::msg::JointState>()
{
  return wearable_robot_interfaces::msg::builder::Init_JointState_header();
}

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
