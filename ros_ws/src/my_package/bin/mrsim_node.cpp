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

#include <geometry_msgs/Twist.h>

#include "lidar.h"
#include "robot.h"
#include "types.h"
#include "world.h"

// using namespace std;
// using json = nlohmann::json;



std::vector<int> vels;

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    ROS_INFO("Received cmd_vel: Linear=%f, Angular=%f", linear_vel, angular_vel);

    std::vector<int> vel;
    vel.push_back(linear_vel);
    vel.push_back(angular_vel);

    vels = vel;
}


int main(int argc, char** argv)
{

  World world; // to create the world

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

  double radius = jsonData["items"][0]["radius"].asDouble();


  // std::shared_ptr<World>  world_pointer(&world, [](World*){ });   // is a lambda function
  std::shared_ptr<World> world_pointer = std::make_shared<World>(world);

  Pose robot_pose = Pose::Identity();
  robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
  robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

  Robot robot(radius, world_pointer, robot_pose);


  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_0/odom", 10);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/robot0/cmd_vel", 10, callback);

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
      world.draw();
      cv::waitKey(1);
      world.timeTick(0.08);

      robot.tv = vels[0];
      robot.rv = vels[1];

      // Update the timestamp
      odom.header.stamp = ros::Time::now();

      // Publish the Odometry message
      odom_pub.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}




