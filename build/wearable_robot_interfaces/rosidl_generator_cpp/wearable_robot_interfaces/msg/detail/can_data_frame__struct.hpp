// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wearable_robot_interfaces:msg/CANDataFrame.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__STRUCT_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__STRUCT_HPP_

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
# define DEPRECATED__wearable_robot_interfaces__msg__CANDataFrame __attribute__((deprecated))
#else
# define DEPRECATED__wearable_robot_interfaces__msg__CANDataFrame __declspec(deprecated)
#endif

namespace wearable_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CANDataFrame_
{
  using Type = CANDataFrame_<ContainerAllocator>;

  explicit CANDataFrame_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit CANDataFrame_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    rosidl_runtime_cpp::BoundedVector<uint8_t, 64, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const rosidl_runtime_cpp::BoundedVector<uint8_t, 64, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> *;
  using ConstRawPtr =
    const wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wearable_robot_interfaces__msg__CANDataFrame
    std::shared_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wearable_robot_interfaces__msg__CANDataFrame
    std::shared_ptr<wearable_robot_interfaces::msg::CANDataFrame_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CANDataFrame_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CANDataFrame_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CANDataFrame_

// alias to use template instance with default allocator
using CANDataFrame =
  wearable_robot_interfaces::msg::CANDataFrame_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__CAN_DATA_FRAME__STRUCT_HPP_
