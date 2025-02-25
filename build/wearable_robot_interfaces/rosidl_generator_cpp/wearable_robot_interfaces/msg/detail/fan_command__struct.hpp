// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wearable_robot_interfaces:msg/FanCommand.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__STRUCT_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__wearable_robot_interfaces__msg__FanCommand __attribute__((deprecated))
#else
# define DEPRECATED__wearable_robot_interfaces__msg__FanCommand __declspec(deprecated)
#endif

namespace wearable_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FanCommand_
{
  using Type = FanCommand_<ContainerAllocator>;

  explicit FanCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit FanCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fan_type =
    std::vector<bool, typename ContainerAllocator::template rebind<bool>::other>;
  _fan_type fan;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fan(
    const std::vector<bool, typename ContainerAllocator::template rebind<bool>::other> & _arg)
  {
    this->fan = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wearable_robot_interfaces__msg__FanCommand
    std::shared_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wearable_robot_interfaces__msg__FanCommand
    std::shared_ptr<wearable_robot_interfaces::msg::FanCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FanCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fan != other.fan) {
      return false;
    }
    return true;
  }
  bool operator!=(const FanCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FanCommand_

// alias to use template instance with default allocator
using FanCommand =
  wearable_robot_interfaces::msg::FanCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__FAN_COMMAND__STRUCT_HPP_
