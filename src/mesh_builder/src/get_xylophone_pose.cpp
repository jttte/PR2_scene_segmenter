#include <ros/ros.h>
#include <ros/package.h>

#include <boost/shared_ptr.hpp>

#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

#include "mesh_builder/MeshCloud.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "get_xylophone_pose_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<mesh_builder::MeshCloud>("get_xylophone_pose");
  mesh_builder::MeshCloud srv;
  //no request input
  if (client.call(srv))
  {

    ROS_INFO("x: %f", (float)srv.response.pose.position.x);
  }
  else
  {
    ROS_ERROR("Failed to call service get_xylophone_pose");
    return 1;
  }

  return 0;
}
