
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

  RobotDriver driver(nh);
  //driver.driveForwardOdom(0.42);
  //driver.turnOdom(true,3.14);
  driver.driveBackwardOdom(0.3);
  //driver.turnOdom(true,3.14);
}
