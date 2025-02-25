// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wearable_robot_interfaces:msg/IMURawData.idl
// generated code does not contain a copyright notice

#ifndef WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_HPP_
#define WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_HPP_

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
// Member 'imu1'
// Member 'imu2'
// Member 'imu3'
// Member 'imu4'
#include "wearable_robot_interfaces/msg/detail/imu_type__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wearable_robot_interfaces__msg__IMURawData __attribute__((deprecated))
#else
# define DEPRECATED__wearable_robot_interfaces__msg__IMURawData __declspec(deprecated)
#endif

namespace wearable_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IMURawData_
{
  using Type = IMURawData_<ContainerAllocator>;

  explicit IMURawData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    imu1(_init),
    imu2(_init),
    imu3(_init),
    imu4(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0ul;
    }
  }

  explicit IMURawData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    imu1(_alloc, _init),
    imu2(_alloc, _init),
    imu3(_alloc, _init),
    imu4(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _can_id_type =
    uint32_t;
  _can_id_type can_id;
  using _imu1_type =
    wearable_robot_interfaces::msg::IMUType_<ContainerAllocator>;
  _imu1_type imu1;
  using _imu2_type =
    wearable_robot_interfaces::msg::IMUType_<ContainerAllocator>;
  _imu2_type imu2;
  using _imu3_type =
    wearable_robot_interfaces::msg::IMUType_<ContainerAllocator>;
  _imu3_type imu3;
  using _imu4_type =
    wearable_robot_interfaces::msg::IMUType_<ContainerAllocator>;
  _imu4_type imu4;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__can_id(
    const uint32_t & _arg)
  {
    this->can_id = _arg;
    return *this;
  }
  Type & set__imu1(
    const wearable_robot_interfaces::msg::IMUType_<ContainerAllocator> & _arg)
  {
    this->imu1 = _arg;
    return *this;
  }
  Type & set__imu2(
    const wearable_robot_interfaces::msg::IMUType_<ContainerAllocator> & _arg)
  {
    this->imu2 = _arg;
    return *this;
  }
  Type & set__imu3(
    const wearable_robot_interfaces::msg::IMUType_<ContainerAllocator> & _arg)
  {
    this->imu3 = _arg;
    return *this;
  }
  Type & set__imu4(
    const wearable_robot_interfaces::msg::IMUType_<ContainerAllocator> & _arg)
  {
    this->imu4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> *;
  using ConstRawPtr =
    const wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wearable_robot_interfaces__msg__IMURawData
    std::shared_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wearable_robot_interfaces__msg__IMURawData
    std::shared_ptr<wearable_robot_interfaces::msg::IMURawData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IMURawData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->imu1 != other.imu1) {
      return false;
    }
    if (this->imu2 != other.imu2) {
      return false;
    }
    if (this->imu3 != other.imu3) {
      return false;
    }
    if (this->imu4 != other.imu4) {
      return false;
    }
    return true;
  }
  bool operator!=(const IMURawData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IMURawData_

// alias to use template instance with default allocator
using IMURawData =
  wearable_robot_interfaces::msg::IMURawData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wearable_robot_interfaces

#endif  // WEARABLE_ROBOT_INTERFACES__MSG__DETAIL__IMU_RAW_DATA__STRUCT_HPP_
