#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"


#include "scene_segmenter/XylophonePose.h"

#include "move_base.h"
#include "math.h"


int main(int argc, char **argv)
{
  float des_x = -0.0910179;
  float des_y = 0.92953;
  float des_z = 2.34948;


  ros::init(argc, argv, "get_xylophone_pose_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<scene_segmenter::XylophonePose>("get_xylophone_pose");

  scene_segmenter::XylophonePose srv;
  bool done = false;
  while(!done) {
    if (client.call(srv))
    {

        ROS_INFO("get service");
        std::cout<<"get pose x: "<<srv.response.pose.position.x<<" y: "<<srv.response.pose.position.y<<" z: "<<srv.response.pose.position.y<<std::endl;
    } else {
        ROS_ERROR("Failed to call service get_xylophone_pose");
        return 1;
    }

    ros::init(argc, argv, "robot_driver");
    RobotDriver driver(nh);
    if(srv.response.pose.position.x < des_x) {
        driver.turnOdom(false, M_PI/2);
        driver.driveForwardOdom(des_x-srv.response.pose.position.x+0.2);
        driver.turnOdom(true, M_PI/2);
    }
    if(srv.response.pose.position.z > des_z) {
        driver.driveForwardOdom(srv.response.pose.position.z-des_z+0.8);
    } 
    done = true;
  }//end of while


  return 0;
}
