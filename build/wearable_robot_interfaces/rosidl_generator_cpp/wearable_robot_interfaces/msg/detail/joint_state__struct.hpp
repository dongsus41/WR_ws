// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wearable_robot_interfaces:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wearable_robot_interfaces__msg__JointState __attribute__((deprecated))
#else
# define DEPRECATED__wearable_robot_interfaces__msg__JointState __declspec(deprecated)
#endif

namespace wearable_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointState_
{
  using Type = JointState_<ContainerAllocator>;

  explicit JointState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r_shoulder_angle = 0.0;
      this->l_shoulder_angle = 0.0;
      this->r_elbow_angle = 0.0;
      this->l_elbow_angle = 0.0;
    }
  }

  explicit JointState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r_shoulder_angle = 0.0;
      this->l_shoulder_angle = 0.0;
      this->r_elbow_angle = 0.0;
      this->l_elbow_angle = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _r_shoulder_angle_type =
    double;
  _r_shoulder_angle_type r_shoulder_angle;
  using _l_shoulder_angle_type =
    double;
  _l_shoulder_angle_type l_shoulder_angle;
  using _r_elbow_angle_type =
    double;
  _r_elbow_angle_type r_elbow_angle;
  using _l_elbow_angle_type =
    double;
  _l_elbow_angle_type l_elbow_angle;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__r_shoulder_angle(
    const double & _arg)
  {
    this->r_shoulder_angle = _arg;
    return *this;
  }
  Type & set__l_shoulder_angle(
    const double & _arg)
  {
    this->l_shoulder_angle = _arg;
    return *this;
  }
  Type & set__r_elbow_angle(
    const double & _arg)
  {
    this->r_elbow_angle = _arg;
    return *this;
  }
  Type & set__l_elbow_angle(
    const double & _arg)
  {
    this->l_elbow_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wearable_robot_interfaces::msg::JointState_<ContainerAllocator> *;
  using ConstRawPtr =
    const wearable_robot_interfaces::msg::JointState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::JointState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::JointState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wearable_robot_interfaces__msg__JointState
    std::shared_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wearable_robot_interfaces__msg__JointState
    std::shared_ptr<wearable_robot_interfaces::msg::JointState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->r_shoulder_angle != other.r_shoulder_angle) {
      return false;
    }
    if (this->l_shoulder_angle != other.l_shoulder_angle) {
      return false;
    }
    if (this->r_elbow_angle != other.r_elbow_angle) {
      return false;
    }
    if (this->l_elbow_angle != other.l_elbow_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointState_

// alias to use template instance with default allocator
using JointState =
  wearable_robot_interfaces::msg::JointState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__JOINT_STATE__STRUCT_HPP_
