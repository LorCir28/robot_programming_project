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

#include "my_package/Num.h"

double vels[2];

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    ROS_INFO("Received cmd_vel: Linear=%f, Angular=%f", linear_vel, angular_vel);


    vels[0] = linear_vel;
    vels[1] = angular_vel;


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
  double initialLinearVelocity = 0;
  double initialAngularVelocity = 0;

  double radius = jsonData["items"][0]["radius"].asDouble();

  double fov = jsonData["items"][1]["fov"].asDouble();
  double max_range = jsonData["items"][1]["max_range"].asDouble();
  double num_beams = jsonData["items"][1]["num_beams"].asDouble();
  double lidarinitialX = jsonData["items"][1]["pose"][0].asDouble();
  double lidarinitialY = jsonData["items"][1]["pose"][1].asDouble();
  double lidarinitialTheta = jsonData["items"][1]["pose"][2].asDouble();


  std::shared_ptr<World> world_pointer(&world, [](World*){ });


  Pose robot_pose = Pose::Identity();
  robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
  robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

  Robot* robot = new Robot(radius, world_pointer, robot_pose);

  std::shared_ptr<Robot> robot_pointer(robot, [](Robot*){ }); 
  Pose lidar_pose = Pose::Identity();
  lidar_pose.translation() = world.grid2world(Eigen::Vector2i(lidarinitialX, lidarinitialY));
  lidar_pose.linear() = Eigen::Rotation2Df(lidarinitialTheta).matrix();

  Lidar* lidar = new Lidar(fov, max_range, num_beams, robot_pointer, lidar_pose);


  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_0/odom", 10);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/robot_0/cmd_vel", 10, callback);
  ros::Publisher lidar_pub = nh.advertise<my_package::Num>("/robot_0/scan", 10);

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


  // Create the customized message
  my_package::Num ranges;


  ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz


  std::string map_path = jsonData["map"].asString();
  world.loadFromImage("/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/test_data/" + map_path); // to load the map image


  world.draw();
  cv::waitKey(1);

  while (ros::ok())
  {
      ros::spinOnce();

      robot->tv = vels[0];
      robot->rv = vels[1];


      world.timeTick(0.08);

      world.draw();
      cv::waitKey(1);

      sleep(0.01);


      // Update the timestamp
      odom.header.stamp = ros::Time::now();

    // to upgrade the odometry of the robot
      odom.pose.pose.position.x = robot->poseInWorld().translation().x();
      odom.pose.pose.position.y = robot->poseInWorld().translation().y();

      Eigen::Rotation2Df rotation(robot->poseInWorld().linear());
      float theta = rotation.angle();

      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

      odom.twist.twist.linear.x = robot->tv;
      odom.twist.twist.angular.z = robot->rv;

      ranges.ranges = lidar->ranges;

      // Publish the Odometry message
      odom_pub.publish(odom);

      // Publish the Lidar message
      lidar_pub.publish(ranges);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}




