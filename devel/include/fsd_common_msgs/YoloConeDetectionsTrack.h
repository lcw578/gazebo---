// Generated by gencpp from file fsd_common_msgs/YoloConeDetectionsTrack.msg
// DO NOT EDIT!


#ifndef FSD_COMMON_MSGS_MESSAGE_YOLOCONEDETECTIONSTRACK_H
#define FSD_COMMON_MSGS_MESSAGE_YOLOCONEDETECTIONSTRACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <fsd_common_msgs/YoloConeTrack.h>

namespace fsd_common_msgs
{
template <class ContainerAllocator>
struct YoloConeDetectionsTrack_
{
  typedef YoloConeDetectionsTrack_<ContainerAllocator> Type;

  YoloConeDetectionsTrack_()
    : header()
    , cone_detections()  {
    }
  YoloConeDetectionsTrack_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cone_detections(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::fsd_common_msgs::YoloConeTrack_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::fsd_common_msgs::YoloConeTrack_<ContainerAllocator> >> _cone_detections_type;
  _cone_detections_type cone_detections;





  typedef boost::shared_ptr< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> const> ConstPtr;

}; // struct YoloConeDetectionsTrack_

typedef ::fsd_common_msgs::YoloConeDetectionsTrack_<std::allocator<void> > YoloConeDetectionsTrack;

typedef boost::shared_ptr< ::fsd_common_msgs::YoloConeDetectionsTrack > YoloConeDetectionsTrackPtr;
typedef boost::shared_ptr< ::fsd_common_msgs::YoloConeDetectionsTrack const> YoloConeDetectionsTrackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator1> & lhs, const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cone_detections == rhs.cone_detections;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator1> & lhs, const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fsd_common_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e97881ef9e11eb60ad6c895ff672af98";
  }

  static const char* value(const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe97881ef9e11eb60ULL;
  static const uint64_t static_value2 = 0xad6c895ff672af98ULL;
};

template<class ContainerAllocator>
struct DataType< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fsd_common_msgs/YoloConeDetectionsTrack";
  }

  static const char* value(const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"fsd_common_msgs/YoloConeTrack[] cone_detections\n"
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
"\n"
"================================================================================\n"
"MSG: fsd_common_msgs/YoloConeTrack\n"
"std_msgs/Float32 x                  # center x\n"
"std_msgs/Float32 y                  # center y\n"
"std_msgs/Float32 width              # width\n"
"std_msgs/Float32 height             # height\n"
"\n"
"std_msgs/String color                 # color of cone, 'r' = red, 'b' = blue, 'y' = yellow\n"
"std_msgs/Float32 colorConfidence   	      # confidence of cone detect\n"
"\n"
"geometry_msgs/Point position # 3d position with (x,y,z)\n"
"std_msgs/Float32 poseConfidence\n"
"\n"
"std_msgs/UInt8 tracking_id\n"
"std_msgs/String tracking_state # OFF, OK, SEARCHING, TERMINATE\n"
"std_msgs/UInt8 missing_frams\n"
"================================================================================\n"
"MSG: std_msgs/Float32\n"
"float32 data\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/UInt8\n"
"uint8 data\n"
;
  }

  static const char* value(const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cone_detections);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct YoloConeDetectionsTrack_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fsd_common_msgs::YoloConeDetectionsTrack_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cone_detections[]" << std::endl;
    for (size_t i = 0; i < v.cone_detections.size(); ++i)
    {
      s << indent << "  cone_detections[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::fsd_common_msgs::YoloConeTrack_<ContainerAllocator> >::stream(s, indent + "    ", v.cone_detections[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FSD_COMMON_MSGS_MESSAGE_YOLOCONEDETECTIONSTRACK_H
