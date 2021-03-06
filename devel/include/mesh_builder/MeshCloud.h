/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/cl3295/robot-test/src/mesh_builder-master/srv/MeshCloud.srv
 *
 */


#ifndef MESH_BUILDER_MESSAGE_MESHCLOUD_H
#define MESH_BUILDER_MESSAGE_MESHCLOUD_H

#include <ros/service_traits.h>


#include <mesh_builder/MeshCloudRequest.h>
#include <mesh_builder/MeshCloudResponse.h>


namespace mesh_builder
{

struct MeshCloud
{

typedef MeshCloudRequest Request;
typedef MeshCloudResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MeshCloud
} // namespace mesh_builder


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mesh_builder::MeshCloud > {
  static const char* value()
  {
    return "57bb768f743be4bd0d6adcabc8b3a329";
  }

  static const char* value(const ::mesh_builder::MeshCloud&) { return value(); }
};

template<>
struct DataType< ::mesh_builder::MeshCloud > {
  static const char* value()
  {
    return "mesh_builder/MeshCloud";
  }

  static const char* value(const ::mesh_builder::MeshCloud&) { return value(); }
};


// service_traits::MD5Sum< ::mesh_builder::MeshCloudRequest> should match 
// service_traits::MD5Sum< ::mesh_builder::MeshCloud > 
template<>
struct MD5Sum< ::mesh_builder::MeshCloudRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mesh_builder::MeshCloud >::value();
  }
  static const char* value(const ::mesh_builder::MeshCloudRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mesh_builder::MeshCloudRequest> should match 
// service_traits::DataType< ::mesh_builder::MeshCloud > 
template<>
struct DataType< ::mesh_builder::MeshCloudRequest>
{
  static const char* value()
  {
    return DataType< ::mesh_builder::MeshCloud >::value();
  }
  static const char* value(const ::mesh_builder::MeshCloudRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mesh_builder::MeshCloudResponse> should match 
// service_traits::MD5Sum< ::mesh_builder::MeshCloud > 
template<>
struct MD5Sum< ::mesh_builder::MeshCloudResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mesh_builder::MeshCloud >::value();
  }
  static const char* value(const ::mesh_builder::MeshCloudResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mesh_builder::MeshCloudResponse> should match 
// service_traits::DataType< ::mesh_builder::MeshCloud > 
template<>
struct DataType< ::mesh_builder::MeshCloudResponse>
{
  static const char* value()
  {
    return DataType< ::mesh_builder::MeshCloud >::value();
  }
  static const char* value(const ::mesh_builder::MeshCloudResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MESH_BUILDER_MESSAGE_MESHCLOUD_H
