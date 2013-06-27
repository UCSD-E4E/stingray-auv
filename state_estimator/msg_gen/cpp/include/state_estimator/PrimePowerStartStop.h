/* Auto-generated by genmsg_cpp for file /home/uwstereo/fuerte_workspace/sandbox/stingray-2-0/state_estimator/msg/PrimePowerStartStop.msg */
#ifndef STATE_ESTIMATOR_MESSAGE_PRIMEPOWERSTARTSTOP_H
#define STATE_ESTIMATOR_MESSAGE_PRIMEPOWERSTARTSTOP_H
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

namespace state_estimator
{
template <class ContainerAllocator>
struct PrimePowerStartStop_ {
  typedef PrimePowerStartStop_<ContainerAllocator> Type;

  PrimePowerStartStop_()
  : header()
  , primePowerStartStop(false)
  {
  }

  PrimePowerStartStop_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , primePowerStartStop(false)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _primePowerStartStop_type;
  uint8_t primePowerStartStop;


  typedef boost::shared_ptr< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator::PrimePowerStartStop_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PrimePowerStartStop
typedef  ::state_estimator::PrimePowerStartStop_<std::allocator<void> > PrimePowerStartStop;

typedef boost::shared_ptr< ::state_estimator::PrimePowerStartStop> PrimePowerStartStopPtr;
typedef boost::shared_ptr< ::state_estimator::PrimePowerStartStop const> PrimePowerStartStopConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::state_estimator::PrimePowerStartStop_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace state_estimator

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::state_estimator::PrimePowerStartStop_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c1703c7a390022e30f74bcfbb99f9be";
  }

  static const char* value(const  ::state_estimator::PrimePowerStartStop_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8c1703c7a390022eULL;
  static const uint64_t static_value2 = 0x30f74bcfbb99f9beULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "state_estimator/PrimePowerStartStop";
  }

  static const char* value(const  ::state_estimator::PrimePowerStartStop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
bool primePowerStartStop\n\
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

  static const char* value(const  ::state_estimator::PrimePowerStartStop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::state_estimator::PrimePowerStartStop_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.primePowerStartStop);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PrimePowerStartStop_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator::PrimePowerStartStop_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::state_estimator::PrimePowerStartStop_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "primePowerStartStop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.primePowerStartStop);
  }
};


} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MESSAGE_PRIMEPOWERSTARTSTOP_H

