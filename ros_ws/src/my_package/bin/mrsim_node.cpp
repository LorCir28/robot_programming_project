#include <ros/ros.h>

#include <opencv2/highgui.hpp>








#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// #include <nlohmann/json.hpp>
#include <fstream>
// #include <json/value.h>
#include <iostream>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
#include <string>

// using namespace std;
// using json = nlohmann::json;






int main(int argc, char** argv)
{
  // Specify the path to the JSON file
  const std::string jsonFilePath = "/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/test_data/cappero_1r.json";

  // Read the JSON file
  std::ifstream jsonFile(jsonFilePath);

  // Check if the file is opened successfully
  if (!jsonFile.is_open()) {
      ROS_ERROR("Failed to open JSON file.");
      return 1;
  }


  // Parse the JSON data
  Json::CharReaderBuilder builder;
  Json::CharReader* reader(builder.newCharReader());
  Json::Value jsonData;

  std::string errors;
  Json::parseFromStream(builder, jsonFile, &jsonData, &errors);

  // Close the file
  jsonFile.close();

  // Extract initial odometry data from the JSON object
  double initialX = jsonData["items"][0]["pose"][0].asDouble();
  double initialY = jsonData["items"][0]["pose"][1].asDouble();
  double initialTheta = jsonData["items"][0]["pose"][2].asDouble();
  double initialLinearVelocity = jsonData["items"][0]["max_tv"].asDouble();
  double initialAngularVelocity = jsonData["items"][0]["max_rv"].asDouble();

  // double initialX = 0;
  // double initialY = 0;
  // double initialTheta = 0;
  // double initialLinearVelocity = 0;
  // double initialAngularVelocity = 0;


  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_0/odom", 10);

  // Create an Odometry message
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // Set the initial pose and velocity using data from JSON
  odom.pose.pose.position.x = initialX;
  odom.pose.pose.position.y = initialY;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initialTheta);

  odom.twist.twist.linear.x = initialLinearVelocity;
  odom.twist.twist.angular.z = initialAngularVelocity;


  ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz

  while (ros::ok())
  {
      // Update the timestamp
      odom.header.stamp = ros::Time::now();

      // Publish the Odometry message
      odom_pub.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}




