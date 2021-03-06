/* Auto-generated by genmsg_cpp for file /home/perry/StingrayProject/stingray-2-0/quat_to_euler/msg/Eulers.msg */
#ifndef QUAT_TO_EULER_MESSAGE_EULERS_H
#define QUAT_TO_EULER_MESSAGE_EULERS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace quat_to_euler
{
template <class ContainerAllocator>
struct Eulers_ {
  typedef Eulers_<ContainerAllocator> Type;

  Eulers_()
  : header()
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  {
  }

  Eulers_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _roll_type;
  double roll;

  typedef double _pitch_type;
  double pitch;

  typedef double _yaw_type;
  double yaw;


  typedef boost::shared_ptr< ::quat_to_euler::Eulers_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quat_to_euler::Eulers_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Eulers
typedef  ::quat_to_euler::Eulers_<std::allocator<void> > Eulers;

typedef boost::shared_ptr< ::quat_to_euler::Eulers> EulersPtr;
typedef boost::shared_ptr< ::quat_to_euler::Eulers const> EulersConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quat_to_euler::Eulers_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quat_to_euler::Eulers_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quat_to_euler

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quat_to_euler::Eulers_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quat_to_euler::Eulers_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quat_to_euler::Eulers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8061e110314ddf08ca1dfbc48d314df8";
  }

  static const char* value(const  ::quat_to_euler::Eulers_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8061e110314ddf08ULL;
  static const uint64_t static_value2 = 0xca1dfbc48d314df8ULL;
};

template<class ContainerAllocator>
struct DataType< ::quat_to_euler::Eulers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quat_to_euler/Eulers";
  }

  static const char* value(const  ::quat_to_euler::Eulers_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quat_to_euler::Eulers_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::quat_to_euler::Eulers_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quat_to_euler::Eulers_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quat_to_euler::Eulers_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quat_to_euler::Eulers_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Eulers_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quat_to_euler::Eulers_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quat_to_euler::Eulers_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUAT_TO_EULER_MESSAGE_EULERS_H

