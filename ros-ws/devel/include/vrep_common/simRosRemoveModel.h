// Generated by gencpp from file vrep_common/simRosRemoveModel.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSREMOVEMODEL_H
#define VREP_COMMON_MESSAGE_SIMROSREMOVEMODEL_H

#include <ros/service_traits.h>


#include <vrep_common/simRosRemoveModelRequest.h>
#include <vrep_common/simRosRemoveModelResponse.h>


namespace vrep_common
{

struct simRosRemoveModel
{

typedef simRosRemoveModelRequest Request;
typedef simRosRemoveModelResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simRosRemoveModel
} // namespace vrep_common


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vrep_common::simRosRemoveModel > {
  static const char* value()
  {
    return "6833353cd429b83819dab869600ce745";
  }

  static const char* value(const ::vrep_common::simRosRemoveModel&) { return value(); }
};

template<>
struct DataType< ::vrep_common::simRosRemoveModel > {
  static const char* value()
  {
    return "vrep_common/simRosRemoveModel";
  }

  static const char* value(const ::vrep_common::simRosRemoveModel&) { return value(); }
};


// service_traits::MD5Sum< ::vrep_common::simRosRemoveModelRequest> should match 
// service_traits::MD5Sum< ::vrep_common::simRosRemoveModel > 
template<>
struct MD5Sum< ::vrep_common::simRosRemoveModelRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosRemoveModel >::value();
  }
  static const char* value(const ::vrep_common::simRosRemoveModelRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosRemoveModelRequest> should match 
// service_traits::DataType< ::vrep_common::simRosRemoveModel > 
template<>
struct DataType< ::vrep_common::simRosRemoveModelRequest>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosRemoveModel >::value();
  }
  static const char* value(const ::vrep_common::simRosRemoveModelRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vrep_common::simRosRemoveModelResponse> should match 
// service_traits::MD5Sum< ::vrep_common::simRosRemoveModel > 
template<>
struct MD5Sum< ::vrep_common::simRosRemoveModelResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosRemoveModel >::value();
  }
  static const char* value(const ::vrep_common::simRosRemoveModelResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosRemoveModelResponse> should match 
// service_traits::DataType< ::vrep_common::simRosRemoveModel > 
template<>
struct DataType< ::vrep_common::simRosRemoveModelResponse>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosRemoveModel >::value();
  }
  static const char* value(const ::vrep_common::simRosRemoveModelResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSREMOVEMODEL_H