# COMS 6998-03 HUMANOID ROBOTS, SPRING 2015
# Columbia University

(Part of ) Final Project
!!ongoing!!

## Getting Started

The following lines will not work unless ROS is properly installed.  You can find instructions for how to do this from the ros_tutorial pdf on the class website.

```bash
$ cd scene_segmenter
$ source /devel/setup.bash
$ catkin_make
```

## bring up Gazebo and the PR2
```bash
$ roslaunch system_launch pr2_gazebo.launch
```

Run the scene segmenter node, which subscribes to kinect and constantly updates the point cloud
```bash
$ rosrun scene_segmenter scene_segmenter_node
```
Run the scene client node, which calls service of scene_segmenter_node and then move PR2 to the front of the biggest object on table
```bash
$ rosrun scene_segmenter scene_client_node
```