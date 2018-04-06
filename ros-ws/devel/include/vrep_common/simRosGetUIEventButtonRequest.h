// Generated by gencpp from file vrep_common/simRosGetUIEventButtonRequest.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSGETUIEVENTBUTTONREQUEST_H
#define VREP_COMMON_MESSAGE_SIMROSGETUIEVENTBUTTONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetUIEventButtonRequest_
{
  typedef simRosGetUIEventButtonRequest_<ContainerAllocator> Type;

  simRosGetUIEventButtonRequest_()
    : uiHandle(0)  {
    }
  simRosGetUIEventButtonRequest_(const ContainerAllocator& _alloc)
    : uiHandle(0)  {
  (void)_alloc;
    }



   typedef int32_t _uiHandle_type;
  _uiHandle_type uiHandle;





  typedef boost::shared_ptr< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> const> ConstPtr;

}; // struct simRosGetUIEventButtonRequest_

typedef ::vrep_common::simRosGetUIEventButtonRequest_<std::allocator<void> > simRosGetUIEventButtonRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetUIEventButtonRequest > simRosGetUIEventButtonRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetUIEventButtonRequest const> simRosGetUIEventButtonRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vrep_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'vrep_common': ['/home/alpsark/ros-ws/src/vrep_stack/vrep_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "66b6744fce66e25d77c762900fd5eb01";
  }

  static const char* value(const ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x66b6744fce66e25dULL;
  static const uint64_t static_value2 = 0x77c762900fd5eb01ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrep_common/simRosGetUIEventButtonRequest";
  }

  static const char* value(const ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
int32 uiHandle\n\
";
  }

  static const char* value(const ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.uiHandle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct simRosGetUIEventButtonRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrep_common::simRosGetUIEventButtonRequest_<ContainerAllocator>& v)
  {
    s << indent << "uiHandle: ";
    Printer<int32_t>::stream(s, indent + "  ", v.uiHandle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSGETUIEVENTBUTTONREQUEST_H
