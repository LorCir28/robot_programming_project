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
// #include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


// publisher
int main(int argc, char** argv) {
  ros::init(argc, argv, "Publisher");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("robot_0/odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);

  while(ros::ok()) {
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }


  // ros::Rate loop_rate(1);

  // while(ros::ok()) {
  //   std_msgs::String msg;
  //   msg.data = "Hello World!!";

  //   pub.publish(msg);
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  
  return 0;
}