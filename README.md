# Columbia University COMS 6998-03 HUMANOID ROBOTS final project, SPRING 2015

## Running the code
bring up Gazebo and the PR2 (Moveit, too. but it's not used in this part of the project)
```bash
$ roslaunch system_launch pr2_gazebo.launch
```

Then run the demo with :
```bash
$ rosrun scene_segmenter scene_segmenter_node
```

The subscriber in this node constantly listerns to "/head_mount_kinect/depth/points" and get the point cloud from kinect. We then segment the point cloud and analyze it with PCL.