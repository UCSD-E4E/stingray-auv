/* Auto-generated by genmsg_cpp for file /home/uwstereo/fuerte_workspace/sandbox/stingray-2-0/microstrain/srv/AddOffset.srv */
#ifndef MICROSTRAIN_SERVICE_ADDOFFSET_H
#define MICROSTRAIN_SERVICE_ADDOFFSET_H
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

#include "ros/service_traits.h"




namespace microstrain
{
template <class ContainerAllocator>
struct AddOffsetRequest_ {
  typedef AddOffsetRequest_<ContainerAllocator> Type;

  AddOffsetRequest_()
  : add_offset(0.0)
  {
  }

  AddOffsetRequest_(const ContainerAllocator& _alloc)
  : add_offset(0.0)
  {
  }

  typedef double _add_offset_type;
  double add_offset;


  typedef boost::shared_ptr< ::microstrain::AddOffsetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::microstrain::AddOffsetRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct AddOffsetRequest
typedef  ::microstrain::AddOffsetRequest_<std::allocator<void> > AddOffsetRequest;

typedef boost::shared_ptr< ::microstrain::AddOffsetRequest> AddOffsetRequestPtr;
typedef boost::shared_ptr< ::microstrain::AddOffsetRequest const> AddOffsetRequestConstPtr;


template <class ContainerAllocator>
struct AddOffsetResponse_ {
  typedef AddOffsetResponse_<ContainerAllocator> Type;

  AddOffsetResponse_()
  : total_offset(0.0)
  {
  }

  AddOffsetResponse_(const ContainerAllocator& _alloc)
  : total_offset(0.0)
  {
  }

  typedef double _total_offset_type;
  double total_offset;


  typedef boost::shared_ptr< ::microstrain::AddOffsetResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::microstrain::AddOffsetResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct AddOffsetResponse
typedef  ::microstrain::AddOffsetResponse_<std::allocator<void> > AddOffsetResponse;

typedef boost::shared_ptr< ::microstrain::AddOffsetResponse> AddOffsetResponsePtr;
typedef boost::shared_ptr< ::microstrain::AddOffsetResponse const> AddOffsetResponseConstPtr;

struct AddOffset
{

typedef AddOffsetRequest Request;
typedef AddOffsetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct AddOffset
} // namespace microstrain

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::microstrain::AddOffsetRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::microstrain::AddOffsetRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::microstrain::AddOffsetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "10fe27c5d4591264b9d05acc7497a18a";
  }

  static const char* value(const  ::microstrain::AddOffsetRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x10fe27c5d4591264ULL;
  static const uint64_t static_value2 = 0xb9d05acc7497a18aULL;
};

template<class ContainerAllocator>
struct DataType< ::microstrain::AddOffsetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "microstrain/AddOffsetRequest";
  }

  static const char* value(const  ::microstrain::AddOffsetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::microstrain::AddOffsetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 add_offset\n\
\n\
";
  }

  static const char* value(const  ::microstrain::AddOffsetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::microstrain::AddOffsetRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::microstrain::AddOffsetResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::microstrain::AddOffsetResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::microstrain::AddOffsetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5dea42ce4656fada4736ce3508b56aca";
  }

  static const char* value(const  ::microstrain::AddOffsetResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5dea42ce4656fadaULL;
  static const uint64_t static_value2 = 0x4736ce3508b56acaULL;
};

template<class ContainerAllocator>
struct DataType< ::microstrain::AddOffsetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "microstrain/AddOffsetResponse";
  }

  static const char* value(const  ::microstrain::AddOffsetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::microstrain::AddOffsetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 total_offset\n\
\n\
\n\
";
  }

  static const char* value(const  ::microstrain::AddOffsetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::microstrain::AddOffsetResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::microstrain::AddOffsetRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.add_offset);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct AddOffsetRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::microstrain::AddOffsetResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.total_offset);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct AddOffsetResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<microstrain::AddOffset> {
  static const char* value() 
  {
    return "f5dcf1246c1a25fcc69616e9d14c1482";
  }

  static const char* value(const microstrain::AddOffset&) { return value(); } 
};

template<>
struct DataType<microstrain::AddOffset> {
  static const char* value() 
  {
    return "microstrain/AddOffset";
  }

  static const char* value(const microstrain::AddOffset&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<microstrain::AddOffsetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5dcf1246c1a25fcc69616e9d14c1482";
  }

  static const char* value(const microstrain::AddOffsetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<microstrain::AddOffsetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "microstrain/AddOffset";
  }

  static const char* value(const microstrain::AddOffsetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<microstrain::AddOffsetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5dcf1246c1a25fcc69616e9d14c1482";
  }

  static const char* value(const microstrain::AddOffsetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<microstrain::AddOffsetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "microstrain/AddOffset";
  }

  static const char* value(const microstrain::AddOffsetResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MICROSTRAIN_SERVICE_ADDOFFSET_H

