#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the cmd_vel topic
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);

    // Create a Twist message with desired linear and angular velocities
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 3.2;     // Linear velocity (m/s)
    cmd_vel_msg.angular.z = 0.5;    // Angular velocity (rad/s)

    ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz

    while (ros::ok())
    {
        cmd_vel_pub.publish(cmd_vel_msg);  // Publish the Twist message
        ros::spinOnce();
        loop_rate.sleep();
    }

    

    // cmd_vel_msg.linear.x = 0;     // Linear velocity (m/s)
    // cmd_vel_msg.angular.z = 0;    // Angular velocity (rad/s)

    // cmd_vel_pub.publish(cmd_vel_msg);  // Publish the Twist message
    // ros::spinOnce();
    // sleep(1);

    // std::cout << "testtstststststs" << std::endl;

    return 0;
}