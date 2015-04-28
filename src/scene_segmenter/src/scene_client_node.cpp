#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
//#include "perception_msgs/SegmentedObject.h"
//#include "perception_msgs/SegmentedObjectList.h"
//#include "perception_msgs/ObjectCenterProperty.h"

#include "cluster_extractor.h"
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

pcl::PCDWriter writer;

void pointcloud_callback(geometry_msgs::Pose pose) {

    //pose.position.x;
    //pose.position.y;
    //pose.position.z;
    std::cout<<"get pose x: "<<pose.position.x<<" y: "<<pose.position.y<<" z: "<<pose.position.z<<std::endl;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "point_cloud_client_node");
  ros::NodeHandle nh;
  // create a templated subscriber
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose> ("segmented_objects", 1, pointcloud_callback);

  ros::spin();

  return 0;
}
