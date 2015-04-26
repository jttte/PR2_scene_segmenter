#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "move_base.h"

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  //float len = argv[1];

  RobotDriver driver(nh);
  driver.turnOdom(true, 1.5707);

}
