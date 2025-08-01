// Generated by gencpp from file fsd_common_msgs/AsState.msg
// DO NOT EDIT!


#ifndef FSD_COMMON_MSGS_MESSAGE_ASSTATE_H
#define FSD_COMMON_MSGS_MESSAGE_ASSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace fsd_common_msgs
{
template <class ContainerAllocator>
struct AsState_
{
  typedef AsState_<ContainerAllocator> Type;

  AsState_()
    : header()
    , mission()
    , whichLap(0)
    , end(0)
    , finished(0)
    , cameraState(0)
    , lidarState(0)
    , insState(0)
    , sensorState(0)
    , ready(0)
    , count_time(0.0)  {
    }
  AsState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , mission(_alloc)
    , whichLap(0)
    , end(0)
    , finished(0)
    , cameraState(0)
    , lidarState(0)
    , insState(0)
    , sensorState(0)
    , ready(0)
    , count_time(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _mission_type;
  _mission_type mission;

   typedef uint8_t _whichLap_type;
  _whichLap_type whichLap;

   typedef uint8_t _end_type;
  _end_type end;

   typedef uint8_t _finished_type;
  _finished_type finished;

   typedef uint8_t _cameraState_type;
  _cameraState_type cameraState;

   typedef uint8_t _lidarState_type;
  _lidarState_type lidarState;

   typedef uint8_t _insState_type;
  _insState_type insState;

   typedef uint8_t _sensorState_type;
  _sensorState_type sensorState;

   typedef uint8_t _ready_type;
  _ready_type ready;

   typedef float _count_time_type;
  _count_time_type count_time;





  typedef boost::shared_ptr< ::fsd_common_msgs::AsState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fsd_common_msgs::AsState_<ContainerAllocator> const> ConstPtr;

}; // struct AsState_

typedef ::fsd_common_msgs::AsState_<std::allocator<void> > AsState;

typedef boost::shared_ptr< ::fsd_common_msgs::AsState > AsStatePtr;
typedef boost::shared_ptr< ::fsd_common_msgs::AsState const> AsStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fsd_common_msgs::AsState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fsd_common_msgs::AsState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fsd_common_msgs::AsState_<ContainerAllocator1> & lhs, const ::fsd_common_msgs::AsState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.mission == rhs.mission &&
    lhs.whichLap == rhs.whichLap &&
    lhs.end == rhs.end &&
    lhs.finished == rhs.finished &&
    lhs.cameraState == rhs.cameraState &&
    lhs.lidarState == rhs.lidarState &&
    lhs.insState == rhs.insState &&
    lhs.sensorState == rhs.sensorState &&
    lhs.ready == rhs.ready &&
    lhs.count_time == rhs.count_time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fsd_common_msgs::AsState_<ContainerAllocator1> & lhs, const ::fsd_common_msgs::AsState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fsd_common_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fsd_common_msgs::AsState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fsd_common_msgs::AsState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fsd_common_msgs::AsState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fsd_common_msgs::AsState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fsd_common_msgs::AsState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fsd_common_msgs::AsState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fsd_common_msgs::AsState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "91d4690a3eeefbcc594048a040c965e0";
  }

  static const char* value(const ::fsd_common_msgs::AsState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x91d4690a3eeefbccULL;
  static const uint64_t static_value2 = 0x594048a040c965e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::fsd_common_msgs::AsState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fsd_common_msgs/AsState";
  }

  static const char* value(const ::fsd_common_msgs::AsState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fsd_common_msgs::AsState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Header\n"
"std_msgs/Header header\n"
"string mission  	#指示当前任务\n"
"uint8 whichLap		#进行记圈，指示当前的圈数\n"
"uint8 end		#指示达到指定停车位置。0表示未到达，1表示到达。随后速度设为0，同时刹车\n"
"#uint8 stop		#指示是否停车。1表示停车，随后通过气瓶进行制动\n"
"uint8 finished		#指示任务完成。0表示未完成，1为任务完成\n"
"uint8 cameraState	#指示相机状态。0为正常，1为断线或异常\n"
"uint8 lidarState	#指示激光雷达状态。0为正常，1为断线或异常\n"
"uint8 insState		#指示组合惯导状态。0为正常，1为断线或异常\n"
"uint8 sensorState	#指示相机、激光雷达、组合惯导状态，同时正常即为0，有一个传感器断线即为1\n"
"uint8 ready		#指示目前无人系统是否准备好，即能否正常接受并处理所有传感器的数据，并正确输出控制指令。0代表正常\n"
"float32 count_time #接收到go信号后开始计时\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::fsd_common_msgs::AsState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fsd_common_msgs::AsState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.mission);
      stream.next(m.whichLap);
      stream.next(m.end);
      stream.next(m.finished);
      stream.next(m.cameraState);
      stream.next(m.lidarState);
      stream.next(m.insState);
      stream.next(m.sensorState);
      stream.next(m.ready);
      stream.next(m.count_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AsState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fsd_common_msgs::AsState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fsd_common_msgs::AsState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mission: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.mission);
    s << indent << "whichLap: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.whichLap);
    s << indent << "end: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.end);
    s << indent << "finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.finished);
    s << indent << "cameraState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cameraState);
    s << indent << "lidarState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lidarState);
    s << indent << "insState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.insState);
    s << indent << "sensorState: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensorState);
    s << indent << "ready: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ready);
    s << indent << "count_time: ";
    Printer<float>::stream(s, indent + "  ", v.count_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FSD_COMMON_MSGS_MESSAGE_ASSTATE_H
