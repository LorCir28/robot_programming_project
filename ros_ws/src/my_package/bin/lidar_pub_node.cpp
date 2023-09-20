#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "lidar_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the base_scan topic
    ros::Publisher base_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/robot0/base_scan", 10);

    // Define LaserScan message
    sensor_msgs::LaserScan laser_scan_msg;

    // Fill in LaserScan message fields
    laser_scan_msg.header.frame_id = "base_laser_link"; // Frame ID for the LiDAR
    laser_scan_msg.angle_min = -M_PI / 2.0; // Minimum scan angle (radians)
    laser_scan_msg.angle_max = M_PI / 2.0;  // Maximum scan angle (radians)
    laser_scan_msg.angle_increment = M_PI / 180.0; // Angle increment between measurements (radians)
    laser_scan_msg.time_increment = 0.0; // Time between measurements (seconds)
    laser_scan_msg.scan_time = 0.1; // Time taken for one scan (seconds)
    laser_scan_msg.range_min = 0.0; // Minimum range value (meters)
    laser_scan_msg.range_max = 10.0; // Maximum range value (meters)

    // Simulated LiDAR data (replace with actual data from your LiDAR)
    int num_measurements = 180;
    laser_scan_msg.ranges.resize(num_measurements);
    for (int i = 0; i < num_measurements; ++i) {
        laser_scan_msg.ranges[i] = static_cast<float>(i) * 0.1; // Simulated range values
    }

    ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz

    while (ros::ok())
    {
        // Publish the LaserScan message
        base_scan_pub.publish(laser_scan_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
