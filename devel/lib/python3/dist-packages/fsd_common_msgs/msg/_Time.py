# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from fsd_common_msgs/Time.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Time(genpy.Message):
  _md5sum = "4cb510b1efb584eb4466d8f45492e730"
  _type = "fsd_common_msgs/Time"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# message of Asensing perception result.It is INS data. 
Header header         # standard ROS message header
uint64 frame_id_num  
float32 sum_compute_time
float32 control_compute_time
float32 boundary_detector_compute_time
float32 line_detector_compute_time
float32 skidpad_detector_compute_time
float32 lidar_detection_compute_time
float32 camera_detection_compute_time
float32 fusion_detection_compute_time
float32 time2
float32 time3
float32 time4
float32 time5
float32 time6
float32 time7
float32 time8
float32 time9


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','frame_id_num','sum_compute_time','control_compute_time','boundary_detector_compute_time','line_detector_compute_time','skidpad_detector_compute_time','lidar_detection_compute_time','camera_detection_compute_time','fusion_detection_compute_time','time2','time3','time4','time5','time6','time7','time8','time9']
  _slot_types = ['std_msgs/Header','uint64','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,frame_id_num,sum_compute_time,control_compute_time,boundary_detector_compute_time,line_detector_compute_time,skidpad_detector_compute_time,lidar_detection_compute_time,camera_detection_compute_time,fusion_detection_compute_time,time2,time3,time4,time5,time6,time7,time8,time9

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Time, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.frame_id_num is None:
        self.frame_id_num = 0
      if self.sum_compute_time is None:
        self.sum_compute_time = 0.
      if self.control_compute_time is None:
        self.control_compute_time = 0.
      if self.boundary_detector_compute_time is None:
        self.boundary_detector_compute_time = 0.
      if self.line_detector_compute_time is None:
        self.line_detector_compute_time = 0.
      if self.skidpad_detector_compute_time is None:
        self.skidpad_detector_compute_time = 0.
      if self.lidar_detection_compute_time is None:
        self.lidar_detection_compute_time = 0.
      if self.camera_detection_compute_time is None:
        self.camera_detection_compute_time = 0.
      if self.fusion_detection_compute_time is None:
        self.fusion_detection_compute_time = 0.
      if self.time2 is None:
        self.time2 = 0.
      if self.time3 is None:
        self.time3 = 0.
      if self.time4 is None:
        self.time4 = 0.
      if self.time5 is None:
        self.time5 = 0.
      if self.time6 is None:
        self.time6 = 0.
      if self.time7 is None:
        self.time7 = 0.
      if self.time8 is None:
        self.time8 = 0.
      if self.time9 is None:
        self.time9 = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.frame_id_num = 0
      self.sum_compute_time = 0.
      self.control_compute_time = 0.
      self.boundary_detector_compute_time = 0.
      self.line_detector_compute_time = 0.
      self.skidpad_detector_compute_time = 0.
      self.lidar_detection_compute_time = 0.
      self.camera_detection_compute_time = 0.
      self.fusion_detection_compute_time = 0.
      self.time2 = 0.
      self.time3 = 0.
      self.time4 = 0.
      self.time5 = 0.
      self.time6 = 0.
      self.time7 = 0.
      self.time8 = 0.
      self.time9 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Q16f().pack(_x.frame_id_num, _x.sum_compute_time, _x.control_compute_time, _x.boundary_detector_compute_time, _x.line_detector_compute_time, _x.skidpad_detector_compute_time, _x.lidar_detection_compute_time, _x.camera_detection_compute_time, _x.fusion_detection_compute_time, _x.time2, _x.time3, _x.time4, _x.time5, _x.time6, _x.time7, _x.time8, _x.time9))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.frame_id_num, _x.sum_compute_time, _x.control_compute_time, _x.boundary_detector_compute_time, _x.line_detector_compute_time, _x.skidpad_detector_compute_time, _x.lidar_detection_compute_time, _x.camera_detection_compute_time, _x.fusion_detection_compute_time, _x.time2, _x.time3, _x.time4, _x.time5, _x.time6, _x.time7, _x.time8, _x.time9,) = _get_struct_Q16f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Q16f().pack(_x.frame_id_num, _x.sum_compute_time, _x.control_compute_time, _x.boundary_detector_compute_time, _x.line_detector_compute_time, _x.skidpad_detector_compute_time, _x.lidar_detection_compute_time, _x.camera_detection_compute_time, _x.fusion_detection_compute_time, _x.time2, _x.time3, _x.time4, _x.time5, _x.time6, _x.time7, _x.time8, _x.time9))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.frame_id_num, _x.sum_compute_time, _x.control_compute_time, _x.boundary_detector_compute_time, _x.line_detector_compute_time, _x.skidpad_detector_compute_time, _x.lidar_detection_compute_time, _x.camera_detection_compute_time, _x.fusion_detection_compute_time, _x.time2, _x.time3, _x.time4, _x.time5, _x.time6, _x.time7, _x.time8, _x.time9,) = _get_struct_Q16f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_Q16f = None
def _get_struct_Q16f():
    global _struct_Q16f
    if _struct_Q16f is None:
        _struct_Q16f = struct.Struct("<Q16f")
    return _struct_Q16f
