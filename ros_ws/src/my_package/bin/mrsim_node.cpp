#include <ros/ros.h>

#include <opencv2/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
#include <string>

#include <geometry_msgs/Twist.h>

#include "lidar.h"
#include "robot.h"
#include "types.h"
#include "world.h"

// #include "my_package/Num.h"
#include "sensor_msgs/LaserScan.h"

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
  const std::string jsonFilePath = "/home/loris/Desktop/university/master/rp/git_project/robot_programming_project/ros_ws/src/my_package/test_data/cappero_1r.json";

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


/////////////////////////////////////////////////////////////////////////////////////////////////////

    int j = 0;
    for (int i = 0; i < jsonData["items"].size(); i++) {
        if (jsonData["items"][i]["type"] == "robot") {
            j++;
        }
    }

    // double initialX[j];
    // double initialY[j];
    // double initialTheta[j];
    // double initialLinearVelocity = 0;
    // double initialAngularVelocity = 0;
    // std::string robot_frame_id[j];
    // double radius[j];
    // std::string robot_namespace[j];
    // std::shared_ptr<World> world_pointer(&world, [](World*){ });
    // Pose robot_pose = Pose::Identity();
    Robot* robots[j];
    nav_msgs::Odometry odoms[j];
    ros::Publisher odom_pubs[j];
    ros::Subscriber cmd_vel_subs[j];



    int k = 0;
    for (int i = 0; i < jsonData["items"].size(); i++) {

        if (jsonData["items"][i]["type"] == "robot") {

            double initialX = jsonData["items"][i]["pose"][0].asDouble();
            double initialY = jsonData["items"][i]["pose"][1].asDouble();
            double initialTheta = jsonData["items"][i]["pose"][2].asDouble();
            double initialLinearVelocity = 0;
            double initialAngularVelocity = 0;
            std::string robot_frame_id = jsonData["items"][i]["frame_id"].asString();
            double radius = jsonData["items"][i]["radius"].asDouble();
            std::string robot_namespace = jsonData["items"][i]["namespace"].asString();

            std::shared_ptr<World> world_pointer(&world, [](World*){ });
            Pose robot_pose = Pose::Identity();
            robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
            robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

            Robot* robot = new Robot(radius, world_pointer, robot_pose);
            robots[k] = robot;

            // std::cout << "11111111" << std::endl;

            // std::shared_ptr<Robot> robot_pointer(robot, [](Robot*){ });

            ros::init(argc, argv, "odometry_publisher");
            ros::NodeHandle nh;
            ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/" + robot_namespace + "/odom", 10);
            ros::Subscriber cmd_vel_sub = nh.subscribe("/" + robot_namespace + "/cmd_vel", 10, callback);

            odom_pubs[k] = odom_pub;
            cmd_vel_subs[k] = cmd_vel_sub;

            // Create an Odometry message
            nav_msgs::Odometry odom;
            //   odom.header.frame_id = "odom";
            odom.header.frame_id = robot_frame_id;
            odom.child_frame_id = "base_link";
            // Set the initial pose and velocity using data from JSON
            odom.pose.pose.position.x = initialX;
            odom.pose.pose.position.y = initialY;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initialTheta);
            odom.twist.twist.linear.x = initialLinearVelocity;
            odom.twist.twist.angular.z = initialAngularVelocity;

            odoms[k] = odom;

            k++;

        }

    }

    // std::cout << "11111111" << std::endl;
    
    ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz


    std::string map_path = jsonData["map"].asString();
    world.loadFromImage("/home/loris/Desktop/university/master/rp/git_project/robot_programming_project/ros_ws/src/my_package/test_data/" + map_path); // to load the map image


    world.draw();
    cv::waitKey(1);

    // std::cout << "22222222" << std::endl;

    while (ros::ok())
    {
        ros::spinOnce();

        // std::cout << k << std::endl;

        for (int i = 0; i < k; i++) {
            robots[i]->tv = vels[0];
            robots[i]->rv = vels[1];
        }

        // std::cout << "22222222" << std::endl;



        world.timeTick(0.08); // world function that calls the timetick functions for all the world items: in this way, the pose of the robot is upgraded

        world.draw();
        cv::waitKey(1);

        sleep(0.01);


        for (int i = 0; i < k; i++) {
            // Update the timestamp
            odoms[i].header.stamp = ros::Time::now();

            // to upgrade the odometry of the robot
            odoms[i].pose.pose.position.x = robots[i]->poseInWorld().translation().x();
            odoms[i].pose.pose.position.y = robots[i]->poseInWorld().translation().y();

            Eigen::Rotation2Df rotation(robots[i]->poseInWorld().linear());
            float theta = rotation.angle();

            odoms[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

            odoms[i].twist.twist.linear.x = robots[i]->tv;
            odoms[i].twist.twist.angular.z = robots[i]->rv;

            //   ranges.ranges = lidar->ranges;

            // Publish the Odometry message
            odom_pubs[i].publish(odoms[i]);

        }

        // to update the lidar data

        // scan_msg.header.stamp = ros::Time::now();
        // scan_msg.ranges = lidar->ranges;


        // Publish the Lidar message
        // lidar_pub.publish(scan_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
  








/////////////////////////////////////////////////////////////////////////////////////////////////////



//   // Extract initial odometry data from the JSON object
//   double initialX = jsonData["items"][0]["pose"][0].asDouble();
//   double initialY = jsonData["items"][0]["pose"][1].asDouble();
//   double initialTheta = jsonData["items"][0]["pose"][2].asDouble();
//   double initialLinearVelocity = 0;
//   double initialAngularVelocity = 0;
//   std::string robot_frame_id = jsonData["items"][0]["frame_id"].asString();
//   double radius = jsonData["items"][0]["radius"].asDouble();

//   double fov = jsonData["items"][1]["fov"].asDouble();
//   double max_range = jsonData["items"][1]["max_range"].asDouble();
//   double num_beams = jsonData["items"][1]["num_beams"].asDouble();
//   double lidarinitialX = jsonData["items"][1]["pose"][0].asDouble();
//   double lidarinitialY = jsonData["items"][1]["pose"][1].asDouble();
//   double lidarinitialTheta = jsonData["items"][1]["pose"][2].asDouble();
//   std::string lidar_frame_id = jsonData["items"][1]["frame_id"].asString();


//   std::shared_ptr<World> world_pointer(&world, [](World*){ });


//   Pose robot_pose = Pose::Identity();
//   robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
//   robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

//   Robot* robot = new Robot(radius, world_pointer, robot_pose);

//   std::shared_ptr<Robot> robot_pointer(robot, [](Robot*){ });

//   Pose lidar_pose = Pose::Identity();
//   lidar_pose.translation() = world.grid2world(Eigen::Vector2i(lidarinitialX, lidarinitialY));
//   lidar_pose.linear() = Eigen::Rotation2Df(lidarinitialTheta).matrix();

//   Lidar* lidar = new Lidar(fov, max_range, num_beams, robot_pointer, lidar_pose);


//   ros::init(argc, argv, "odometry_publisher");
//   ros::NodeHandle nh;
//   ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_0/odom", 10);
//   ros::Subscriber cmd_vel_sub = nh.subscribe("/robot_0/cmd_vel", 10, callback);
// //   ros::Publisher lidar_pub = nh.advertise<my_package::Num>("/robot_0/scan", 10);
//   ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/robot_0/scan", 10);

//   // Create an Odometry message
//   nav_msgs::Odometry odom;
// //   odom.header.frame_id = "odom";
//   odom.header.frame_id = robot_frame_id;
//   odom.child_frame_id = "base_link";

//   // Set the initial pose and velocity using data from JSON
//   odom.pose.pose.position.x = initialX;
//   odom.pose.pose.position.y = initialY;
//   odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initialTheta);

//   odom.twist.twist.linear.x = initialLinearVelocity;
//   odom.twist.twist.angular.z = initialAngularVelocity;


//   // Create the customized message
// //   my_package::Num ranges;
//   sensor_msgs::LaserScan scan_msg;

//   scan_msg.header.frame_id = lidar_frame_id;
//   scan_msg.angle_min = -fov;
//   scan_msg.angle_max = fov;
//   scan_msg.angle_increment = fov / num_beams;
//   scan_msg.time_increment = 0.001;
//   scan_msg.scan_time = 0.1;
//   scan_msg.range_min = 0.0;
//   scan_msg.range_max = max_range;
// //   scan_msg.ranges.resize(num_beams, 2.0);
//   scan_msg.ranges = lidar->ranges;


//   ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz


//   std::string map_path = jsonData["map"].asString();
//   world.loadFromImage("/home/loris/Desktop/university/master/rp/git_project/robot_programming_project/ros_ws/src/my_package/test_data/" + map_path); // to load the map image


//   world.draw();
//   cv::waitKey(1);

//   while (ros::ok())
//   {
//       ros::spinOnce();

//       robot->tv = vels[0];
//       robot->rv = vels[1];


//       world.timeTick(0.08); // world function that calls the timetick functions for all the world items: in this way, the pose of the robot is upgraded

//       world.draw();
//       cv::waitKey(1);

//       sleep(0.01);


//       // Update the timestamp
//       odom.header.stamp = ros::Time::now();

//       // to upgrade the odometry of the robot
//       odom.pose.pose.position.x = robot->poseInWorld().translation().x();
//       odom.pose.pose.position.y = robot->poseInWorld().translation().y();

//       Eigen::Rotation2Df rotation(robot->poseInWorld().linear());
//       float theta = rotation.angle();

//       odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

//       odom.twist.twist.linear.x = robot->tv;
//       odom.twist.twist.angular.z = robot->rv;

//     //   ranges.ranges = lidar->ranges;

//       // Publish the Odometry message
//       odom_pub.publish(odom);

//       // to update the lidar data

//       scan_msg.header.stamp = ros::Time::now();
//       scan_msg.ranges = lidar->ranges;


//       // Publish the Lidar message
//       lidar_pub.publish(scan_msg);

//       ros::spinOnce();
//       loop_rate.sleep();
//   }

  return 0;

}




