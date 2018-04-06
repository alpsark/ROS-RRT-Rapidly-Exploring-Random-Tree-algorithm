// Generated by gencpp from file vrep_common/simRosGetIntegerParameterResponse.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSGETINTEGERPARAMETERRESPONSE_H
#define VREP_COMMON_MESSAGE_SIMROSGETINTEGERPARAMETERRESPONSE_H


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
struct simRosGetIntegerParameterResponse_
{
  typedef simRosGetIntegerParameterResponse_<ContainerAllocator> Type;

  simRosGetIntegerParameterResponse_()
    : result(0)
    , parameterValue(0)  {
    }
  simRosGetIntegerParameterResponse_(const ContainerAllocator& _alloc)
    : result(0)
    , parameterValue(0)  {
  (void)_alloc;
    }



   typedef int32_t _result_type;
  _result_type result;

   typedef int32_t _parameterValue_type;
  _parameterValue_type parameterValue;





  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> const> ConstPtr;

}; // struct simRosGetIntegerParameterResponse_

typedef ::vrep_common::simRosGetIntegerParameterResponse_<std::allocator<void> > simRosGetIntegerParameterResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse > simRosGetIntegerParameterResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetIntegerParameterResponse const> simRosGetIntegerParameterResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6c2f4c807e1ab6d671a7c18b9d47ce4a";
  }

  static const char* value(const ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6c2f4c807e1ab6d6ULL;
  static const uint64_t static_value2 = 0x71a7c18b9d47ce4aULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrep_common/simRosGetIntegerParameterResponse";
  }

  static const char* value(const ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 result\n\
int32 parameterValue\n\
\n\
";
  }

  static const char* value(const ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.parameterValue);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct simRosGetIntegerParameterResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrep_common::simRosGetIntegerParameterResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<int32_t>::stream(s, indent + "  ", v.result);
    s << indent << "parameterValue: ";
    Printer<int32_t>::stream(s, indent + "  ", v.parameterValue);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSGETINTEGERPARAMETERRESPONSE_H
