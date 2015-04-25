#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/projection_matrix.h>
#include "pcl_ros/point_cloud.h"

#include <sensor_msgs/PointCloud2.h>
#include "mesh_builder/MeshCloud.h"
//messages type
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud2.h"



sensor_msgs::PointCloud2 current_cloud;

bool service_callback( mesh_builder::MeshCloud::Request &req,
                      mesh_builder::MeshCloud::Response &res ){
    std::cout<<"receive call";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 1;

    res.pose = pose;

    //sensor_msgs::PointCloud2 processed_cloud = current_cloud;

    return true;

}

void pointcloud_callback(sensor_msgs::PointCloud2 pc) {

    current_cloud = pc;
    std::cout<<"update cloud"<<std::endl;

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh;
  // create a templated subscriber
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/head_mount_kinect/depth/points", 1, pointcloud_callback);

  ros::spin();

  ros::ServiceServer service = nh.advertiseService("get_xylophone_pose", service_callback);
  //pub.publish("test");


  return 0;
}
