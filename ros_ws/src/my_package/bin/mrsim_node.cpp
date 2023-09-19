#include <ros/ros.h>

#include <opencv2/highgui.hpp>

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "mrsim_node");
//   ros::NodeHandle nh("/");

//   // Load the configuration file and initialize the simulator

//   while (ros::ok()) {
//     // run a simulation iteration
//     // visualize the simulation
//     cv::waitKey(100);
//     ros::spinOnce();
//   }

//   return 0;
// }



#include "ros/ros.h"
#include "std_msgs/String.h"


// publisher
int main(int argc, char** argv) {
  ros::init(argc, argv, "Publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::String>("robot_0/odom", 1000);
  ros::Rate loop_rate(1);

  while(ros::ok()) {
    std_msgs::String msg;
    msg.data = "Hello World!!";

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}