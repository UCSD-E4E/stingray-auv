/* Auto-generated by genmsg_cpp for file /home/uwstereo/fuerte_workspace/sandbox/stingray-2-0/state_estimator/msg/State.msg */
#ifndef STATE_ESTIMATOR_MESSAGE_STATE_H
#define STATE_ESTIMATOR_MESSAGE_STATE_H
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
struct State_ {
  typedef State_<ContainerAllocator> Type;

  State_()
  : header()
  , lat(0.0)
  , lon(0.0)
  , depth(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , surge(0.0)
  , sway(0.0)
  , heave(0.0)
  , surge_dot(0.0)
  , sway_dot(0.0)
  , heave_dot(0.0)
  , mag_x(0.0)
  , mag_y(0.0)
  , mag_z(0.0)
  {
  }

  State_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , lat(0.0)
  , lon(0.0)
  , depth(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , surge(0.0)
  , sway(0.0)
  , heave(0.0)
  , surge_dot(0.0)
  , sway_dot(0.0)
  , heave_dot(0.0)
  , mag_x(0.0)
  , mag_y(0.0)
  , mag_z(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _lat_type;
  float lat;

  typedef float _lon_type;
  float lon;

  typedef float _depth_type;
  float depth;

  typedef float _roll_type;
  float roll;

  typedef float _pitch_type;
  float pitch;

  typedef float _yaw_type;
  float yaw;

  typedef float _surge_type;
  float surge;

  typedef float _sway_type;
  float sway;

  typedef float _heave_type;
  float heave;

  typedef float _surge_dot_type;
  float surge_dot;

  typedef float _sway_dot_type;
  float sway_dot;

  typedef float _heave_dot_type;
  float heave_dot;

  typedef float _mag_x_type;
  float mag_x;

  typedef float _mag_y_type;
  float mag_y;

  typedef float _mag_z_type;
  float mag_z;


  typedef boost::shared_ptr< ::state_estimator::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator::State_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct State
typedef  ::state_estimator::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::state_estimator::State> StatePtr;
typedef boost::shared_ptr< ::state_estimator::State const> StateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::state_estimator::State_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::state_estimator::State_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace state_estimator

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::state_estimator::State_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::state_estimator::State_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::state_estimator::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a14638f1c8add4a6304fb5fa95c6157c";
  }

  static const char* value(const  ::state_estimator::State_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa14638f1c8add4a6ULL;
  static const uint64_t static_value2 = 0x304fb5fa95c6157cULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "state_estimator/State";
  }

  static const char* value(const  ::state_estimator::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::state_estimator::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 lat\n\
float32 lon\n\
float32 depth \n\
float32 roll \n\
float32 pitch\n\
float32 yaw\n\
float32 surge\n\
float32 sway\n\
float32 heave\n\
float32 surge_dot\n\
float32 sway_dot\n\
float32 heave_dot\n\
float32 mag_x\n\
float32 mag_y\n\
float32 mag_z\n\
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

  static const char* value(const  ::state_estimator::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::state_estimator::State_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::state_estimator::State_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::state_estimator::State_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.lat);
    stream.next(m.lon);
    stream.next(m.depth);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.surge);
    stream.next(m.sway);
    stream.next(m.heave);
    stream.next(m.surge_dot);
    stream.next(m.sway_dot);
    stream.next(m.heave_dot);
    stream.next(m.mag_x);
    stream.next(m.mag_y);
    stream.next(m.mag_z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct State_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::state_estimator::State_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lat: ";
    Printer<float>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<float>::stream(s, indent + "  ", v.lon);
    s << indent << "depth: ";
    Printer<float>::stream(s, indent + "  ", v.depth);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "surge: ";
    Printer<float>::stream(s, indent + "  ", v.surge);
    s << indent << "sway: ";
    Printer<float>::stream(s, indent + "  ", v.sway);
    s << indent << "heave: ";
    Printer<float>::stream(s, indent + "  ", v.heave);
    s << indent << "surge_dot: ";
    Printer<float>::stream(s, indent + "  ", v.surge_dot);
    s << indent << "sway_dot: ";
    Printer<float>::stream(s, indent + "  ", v.sway_dot);
    s << indent << "heave_dot: ";
    Printer<float>::stream(s, indent + "  ", v.heave_dot);
    s << indent << "mag_x: ";
    Printer<float>::stream(s, indent + "  ", v.mag_x);
    s << indent << "mag_y: ";
    Printer<float>::stream(s, indent + "  ", v.mag_y);
    s << indent << "mag_z: ";
    Printer<float>::stream(s, indent + "  ", v.mag_z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MESSAGE_STATE_H

