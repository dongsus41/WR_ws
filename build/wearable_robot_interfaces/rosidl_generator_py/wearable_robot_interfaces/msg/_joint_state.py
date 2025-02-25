# generated from rosidl_generator_py/resource/_idl.py.em
# with input from wearable_robot_interfaces:msg/JointState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JointState(type):
    """Metaclass of message 'JointState'."""

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
                'wearable_robot_interfaces.msg.JointState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joint_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joint_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joint_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joint_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joint_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class JointState(metaclass=Metaclass_JointState):
    """Message class 'JointState'."""

    __slots__ = [
        '_header',
        '_r_shoulder_angle',
        '_l_shoulder_angle',
        '_r_elbow_angle',
        '_l_elbow_angle',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'r_shoulder_angle': 'double',
        'l_shoulder_angle': 'double',
        'r_elbow_angle': 'double',
        'l_elbow_angle': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.r_shoulder_angle = kwargs.get('r_shoulder_angle', float())
        self.l_shoulder_angle = kwargs.get('l_shoulder_angle', float())
        self.r_elbow_angle = kwargs.get('r_elbow_angle', float())
        self.l_elbow_angle = kwargs.get('l_elbow_angle', float())

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
        if self.r_shoulder_angle != other.r_shoulder_angle:
            return False
        if self.l_shoulder_angle != other.l_shoulder_angle:
            return False
        if self.r_elbow_angle != other.r_elbow_angle:
            return False
        if self.l_elbow_angle != other.l_elbow_angle:
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
    def r_shoulder_angle(self):
        """Message field 'r_shoulder_angle'."""
        return self._r_shoulder_angle

    @r_shoulder_angle.setter
    def r_shoulder_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r_shoulder_angle' field must be of type 'float'"
        self._r_shoulder_angle = value

    @property
    def l_shoulder_angle(self):
        """Message field 'l_shoulder_angle'."""
        return self._l_shoulder_angle

    @l_shoulder_angle.setter
    def l_shoulder_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_shoulder_angle' field must be of type 'float'"
        self._l_shoulder_angle = value

    @property
    def r_elbow_angle(self):
        """Message field 'r_elbow_angle'."""
        return self._r_elbow_angle

    @r_elbow_angle.setter
    def r_elbow_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r_elbow_angle' field must be of type 'float'"
        self._r_elbow_angle = value

    @property
    def l_elbow_angle(self):
        """Message field 'l_elbow_angle'."""
        return self._l_elbow_angle

    @l_elbow_angle.setter
    def l_elbow_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_elbow_angle' field must be of type 'float'"
        self._l_elbow_angle = value
