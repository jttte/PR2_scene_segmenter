#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>


#include <sensor_msgs/PointCloud2.h>
#include "mesh_builder/MeshCloud.h"
//messages type
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud2.h"

#include "cluster_extractor.h"

pcl::PCDWriter writer;
pcl::PCLPointCloud2::Ptr current_cloud;

void process_cloud()
{


        
}

bool service_callback( mesh_builder::MeshCloud::Request &req,
                      mesh_builder::MeshCloud::Response &res ){
    std::cout<<"receive call";

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 1;

    //res.pose = pose;
    //pose = process_cloud();

    return true;

}

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *cloud);
    //std::cout<<msg->header<<std::endl;
    //current_cloud = cloud;
    std::cout<<"update cloud"<<std::endl;
    

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh;
  // create a templated subscriber
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/head_mount_kinect/depth/points", 1, pointcloud_callback);

  
  ros::ServiceServer service = nh.advertiseService("get_xylophone_pose", service_callback);
  ros::spin();
  //current_cloud = new pcl::PCLPointCloud2();

  return 0;
}
