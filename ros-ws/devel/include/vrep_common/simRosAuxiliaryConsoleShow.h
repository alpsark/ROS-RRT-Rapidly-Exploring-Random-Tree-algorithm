// Generated by gencpp from file vrep_common/simRosAuxiliaryConsoleShow.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSAUXILIARYCONSOLESHOW_H
#define VREP_COMMON_MESSAGE_SIMROSAUXILIARYCONSOLESHOW_H

#include <ros/service_traits.h>


#include <vrep_common/simRosAuxiliaryConsoleShowRequest.h>
#include <vrep_common/simRosAuxiliaryConsoleShowResponse.h>


namespace vrep_common
{

struct simRosAuxiliaryConsoleShow
{

typedef simRosAuxiliaryConsoleShowRequest Request;
typedef simRosAuxiliaryConsoleShowResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simRosAuxiliaryConsoleShow
} // namespace vrep_common


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShow > {
  static const char* value()
  {
    return "0441070f5e93cc5cd766aa59d6093443";
  }

  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShow&) { return value(); }
};

template<>
struct DataType< ::vrep_common::simRosAuxiliaryConsoleShow > {
  static const char* value()
  {
    return "vrep_common/simRosAuxiliaryConsoleShow";
  }

  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShow&) { return value(); }
};


// service_traits::MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShowRequest> should match 
// service_traits::MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShow > 
template<>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShowRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShow >::value();
  }
  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShowRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosAuxiliaryConsoleShowRequest> should match 
// service_traits::DataType< ::vrep_common::simRosAuxiliaryConsoleShow > 
template<>
struct DataType< ::vrep_common::simRosAuxiliaryConsoleShowRequest>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosAuxiliaryConsoleShow >::value();
  }
  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShowRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShowResponse> should match 
// service_traits::MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShow > 
template<>
struct MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShowResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosAuxiliaryConsoleShow >::value();
  }
  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShowResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosAuxiliaryConsoleShowResponse> should match 
// service_traits::DataType< ::vrep_common::simRosAuxiliaryConsoleShow > 
template<>
struct DataType< ::vrep_common::simRosAuxiliaryConsoleShowResponse>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosAuxiliaryConsoleShow >::value();
  }
  static const char* value(const ::vrep_common::simRosAuxiliaryConsoleShowResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSAUXILIARYCONSOLESHOW_H