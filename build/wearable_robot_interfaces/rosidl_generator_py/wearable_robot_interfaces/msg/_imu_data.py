# generated from rosidl_generator_py/resource/_idl.py.em
# with input from wearable_robot_interfaces:msg/IMUData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IMUData(type):
    """Metaclass of message 'IMUData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('wearable_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'wearable_robot_interfaces.msg.IMUData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__imu_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__imu_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__imu_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__imu_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__imu_data

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from wearable_robot_interfaces.msg import IMUType
            if IMUType.__class__._TYPE_SUPPORT is None:
                IMUType.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IMUData(metaclass=Metaclass_IMUData):
    """Message class 'IMUData'."""

    __slots__ = [
        '_header',
        '_imu1',
        '_imu2',
        '_imu3',
        '_imu4',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'imu1': 'wearable_robot_interfaces/IMUType',
        'imu2': 'wearable_robot_interfaces/IMUType',
        'imu3': 'wearable_robot_interfaces/IMUType',
        'imu4': 'wearable_robot_interfaces/IMUType',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['wearable_robot_interfaces', 'msg'], 'IMUType'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['wearable_robot_interfaces', 'msg'], 'IMUType'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['wearable_robot_interfaces', 'msg'], 'IMUType'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['wearable_robot_interfaces', 'msg'], 'IMUType'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from wearable_robot_interfaces.msg import IMUType
        self.imu1 = kwargs.get('imu1', IMUType())
        from wearable_robot_interfaces.msg import IMUType
        self.imu2 = kwargs.get('imu2', IMUType())
        from wearable_robot_interfaces.msg import IMUType
        self.imu3 = kwargs.get('imu3', IMUType())
        from wearable_robot_interfaces.msg import IMUType
        self.imu4 = kwargs.get('imu4', IMUType())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.imu1 != other.imu1:
            return False
        if self.imu2 != other.imu2:
            return False
        if self.imu3 != other.imu3:
            return False
        if self.imu4 != other.imu4:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def imu1(self):
        """Message field 'imu1'."""
        return self._imu1

    @imu1.setter
    def imu1(self, value):
        if __debug__:
            from wearable_robot_interfaces.msg import IMUType
            assert \
                isinstance(value, IMUType), \
                "The 'imu1' field must be a sub message of type 'IMUType'"
        self._imu1 = value

    @property
    def imu2(self):
        """Message field 'imu2'."""
        return self._imu2

    @imu2.setter
    def imu2(self, value):
        if __debug__:
            from wearable_robot_interfaces.msg import IMUType
            assert \
                isinstance(value, IMUType), \
                "The 'imu2' field must be a sub message of type 'IMUType'"
        self._imu2 = value

    @property
    def imu3(self):
        """Message field 'imu3'."""
        return self._imu3

    @imu3.setter
    def imu3(self, value):
        if __debug__:
            from wearable_robot_interfaces.msg import IMUType
            assert \
                isinstance(value, IMUType), \
                "The 'imu3' field must be a sub message of type 'IMUType'"
        self._imu3 = value

    @property
    def imu4(self):
        """Message field 'imu4'."""
        return self._imu4

    @imu4.setter
    def imu4(self, value):
        if __debug__:
            from wearable_robot_interfaces.msg import IMUType
            assert \
                isinstance(value, IMUType), \
                "The 'imu4' field must be a sub message of type 'IMUType'"
        self._imu4 = value
