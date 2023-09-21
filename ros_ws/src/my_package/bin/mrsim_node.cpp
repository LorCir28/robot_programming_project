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



// std::vector<int> vels;
double vels[2];

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    ROS_INFO("Received cmd_vel: Linear=%f, Angular=%f", linear_vel, angular_vel);

    // std::vector<int> vel;
    // vel.push_back(linear_vel);
    // vel.push_back(angular_vel);

    // vels = vel;

    vels[0] = linear_vel;
    vels[1] = angular_vel;

    // ROS_INFO("Received cmd_vel_from_vector: Linear=%f, Angular=%f", vels[0], vels[1]);

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
//   double initialLinearVelocity = jsonData["items"][0]["max_tv"].asDouble();
//   double initialAngularVelocity = jsonData["items"][0]["max_rv"].asDouble();
  double initialLinearVelocity = 0;
  double initialAngularVelocity = 0;

  double radius = jsonData["items"][0]["radius"].asDouble();


  std::shared_ptr<World>  world_pointer(&world, [](World*){ });   // is a lambda function
//   std::shared_ptr<World> world_pointer = std::make_shared<World>(world);

  Pose robot_pose = Pose::Identity();
  robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
  robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

  Robot* robot = new Robot(radius, world_pointer, robot_pose);


  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_0/odom", 10);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/robot_0/cmd_vel", 10, callback);

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


  std::string map_path = jsonData["map"].asString();
  world.loadFromImage("/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/test_data/" + map_path); // to load the map image

//   std::cout << "ciaooooo" << world._items.size();

  world.draw();
  cv::waitKey(1);

  while (ros::ok())
  {
      ros::spinOnce();

      robot->tv = vels[0];
      robot->rv = vels[1];

    //   sleep(0.01);

      world.timeTick(0.08);

      world.draw();
      cv::waitKey(1);

    //   robot->tv = 0;
    //   robot->rv = 0;

    //   std::cout << vels[0] << std::endl;

      sleep(0.01);




    //   std::cout << robot->tv << std::endl;

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

      // Publish the Odometry message
      odom_pub.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}




