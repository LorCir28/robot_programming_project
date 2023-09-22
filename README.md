# Project of Robot Programming exam at La Sapienza University of Rome (Artificial intelligence and Robotics master degree)

## Multi-robot simulator in ROS

This project consists of a ROS support for a Multi-robot simulator written in cpp.

There are two ROS nodes i.e.
```sh
mrsim_node.cpp
cmd_vel_pub_node.cpp
```
The first one draws the map, publishes the odometry of the robot, reads the cmd_vel to give to it and puts a lidar on the robot in order to publish its measurements.

The second one publishes the cmd_vel to give to the robot.

### To compile
Do the following command from the catkin workspace directory
```sh
catkin_make
```

### To run and test
All the following commands have to be done in differents shells
```sh
rosrun my_package test_node # to run the mrsim_node.cpp
rosrun my_package cmd_vel_pub # to run the cmd_vel_pub_node.cpp
rostopic list # to visualize the topics
rostopic echo /robot_0/odom # to visualize the odometry of the robot
rostopic echo /robot_0/cmd_vel # to visualize the velocity of the robot
rostopic echo /robot_0/scan # to visualize the lidar measurements
```

In the shell in which you run the cmd_vel_pub node, press the keyboard arrows to move the robot (you will se the robot moving in the map).

By moving the robot, it can be aprreciated that its odometry, its velocity and the lidar measurements change.

