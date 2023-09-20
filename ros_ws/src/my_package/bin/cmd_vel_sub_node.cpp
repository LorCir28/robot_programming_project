#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    ROS_INFO("Received cmd_vel: Linear=%f, Angular=%f", linear_vel, angular_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe("/robot0/cmd_vel", 10, callback);

    ros::spin();

    return 0;
}