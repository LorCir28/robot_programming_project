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
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h> 

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    int t = 0;
    for (int i = 0; i < jsonData["items"].size(); i++) {
        if (jsonData["items"][i]["type"] == "robot") {
            j++;
        }
        else if (jsonData["items"][i]["type"] == "lidar") {
            t++;
        }
    }

    Robot* robots[j];
    nav_msgs::Odometry odoms[j];
    ros::Publisher odom_pubs[j];
    ros::Subscriber cmd_vel_subs[j];
    int robot_ids[j];
    std::string robot_frame_ids[j];

    sensor_msgs::LaserScan scan_msgs[t];
    Lidar* lidars[t];
    ros::Publisher lidar_pubs[t];
    



    int k = 0;
    int r = 0;
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
            int robot_id = jsonData["items"][i]["id"].asInt();

            robot_ids[k] = robot_id;
            robot_frame_ids[k] = robot_frame_id;


            std::shared_ptr<World> world_pointer(&world, [](World*){ });
            Pose robot_pose = Pose::Identity();
            robot_pose.translation() = world.grid2world(Eigen::Vector2i(initialX, initialY));
            robot_pose.linear() = Eigen::Rotation2Df(initialTheta).matrix();

            Robot* robot = new Robot(radius, world_pointer, robot_pose);
            robots[k] = robot;


            ros::init(argc, argv, "odometry_publisher");
            ros::NodeHandle nh;
            ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/" + robot_namespace + "/odom", 10);
            ros::Subscriber cmd_vel_sub = nh.subscribe("/" + robot_namespace + "/cmd_vel", 10, callback);


            odom_pubs[k] = odom_pub;
            cmd_vel_subs[k] = cmd_vel_sub;

            // Create an Odometry message
            nav_msgs::Odometry odom;
            //   odom.header.frame_id = "odom";
            odom.header.frame_id = "map";
            odom.child_frame_id = robot_frame_id;
            // Set the initial pose and velocity using data from JSON
            odom.pose.pose.position.x = initialX;
            odom.pose.pose.position.y = initialY;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initialTheta);
            odom.twist.twist.linear.x = initialLinearVelocity;
            odom.twist.twist.angular.z = initialAngularVelocity;

            odoms[k] = odom;


            // transform odometry sending
            // tf::TransformBroadcaster odom_broadcaster;
            // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(initialTheta);

            // geometry_msgs::TransformStamped odom_trans;
            // odom_trans.header.stamp = ros::Time::now();
            // odom_trans.header.frame_id = "map";
            // odom_trans.child_frame_id = robot_frame_id;

            // odom_trans.transform.translation.x = initialX;
            // odom_trans.transform.translation.y = initialY;
            // // odom_trans.transform.translation.z = 0.0;
            // odom_trans.transform.rotation = odom_quat;

            // //send the transform
            // odom_broadcaster.sendTransform(odom_trans);


            // geometry_msgs::TransformStamped transform_stamped;
            // transform_stamped.header.stamp = ros::Time::now();
            // transform_stamped.header.frame_id = "map";
            // transform_stamped.child_frame_id = robot_frame_id;

            /**
             * 
             * This is the right choice for representing 2D rotations.
             * 
             * Quaternion: x, y, z, w
             */

            // Pose transformation = robot->poseInWorld();
            // tf2::Quaternion rotation;
            // rotation.setRPY(0.0, 0.0, Rotation(transformation.linear()).angle());
            // rotation.normalize();

            // // Let's create a translation vector which will be composed by (x, y, z).
            // // Since we are in 2D, z is equal to 0.
            // tf2::Vector3 translation(transformation.translation().x(), transformation.translation().y(), 0.0);

            // // Now we create the Transform object, by applying the rotation and translation created before.
            // tf2::Transform tf_transform(rotation, translation);

            // // Translation part. We get the translation values and put them in the transform_stamped.
            // transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
            // transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
            // transform_stamped.transform.translation.z = tf_transform.getOrigin().z();

            // // Rotation part. We get the rotation values and put them in the transform_stamped.
            // transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
            // transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
            // transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
            // transform_stamped.transform.rotation.w = tf_transform.getRotation().w();

            // static tf2_ros::TransformBroadcaster br;
            // br.sendTransform(transform_stamped);


            k++;

        }

        else if (jsonData["items"][i]["type"] == "lidar") {

            double fov = jsonData["items"][i]["fov"].asDouble();
            double max_range = jsonData["items"][i]["max_range"].asDouble();
            double num_beams = jsonData["items"][i]["num_beams"].asDouble();
            double lidarinitialX = jsonData["items"][i]["pose"][0].asDouble();
            double lidarinitialY = jsonData["items"][i]["pose"][1].asDouble();
            double lidarinitialTheta = jsonData["items"][i]["pose"][2].asDouble();
            std::string lidar_frame_id = jsonData["items"][i]["frame_id"].asString();
            std::string robot_namespace = jsonData["items"][i]["namespace"].asString();
            int parent_id = jsonData["items"][i]["parent"].asInt();


            Pose lidar_pose = Pose::Identity();
            lidar_pose.translation() = world.grid2world(Eigen::Vector2i(lidarinitialX, lidarinitialY));
            lidar_pose.linear() = Eigen::Rotation2Df(lidarinitialTheta).matrix();


            sensor_msgs::LaserScan scan_msg;
            // geometry_msgs::TransformStamped scan_trans;
            // static tf2_ros::TransformBroadcaster br;
            // geometry_msgs::TransformStamped transform_stamped;
            for (int i = 0; i < sizeof(robots) / sizeof(robots[0]); i++) {

                if (parent_id == robot_ids[i]) {
                    std::shared_ptr<Robot> robot_pointer(robots[i], [](Robot*){ });
                    Lidar* lidar = new Lidar(fov, max_range, num_beams, robot_pointer, lidar_pose);
            
                    lidars[r] = lidar;

                    // scan_trans.header.frame_id = robot_frame_ids[i];
                    // scan_trans.header.frame_id = "map";
                    // scan_msg.header.frame_id = robot_frame_ids[i];
                    scan_msg.header.frame_id = "map";
                    // transform_stamped.header.frame_id = robot_frame_ids[i];
                }


            }


            ros::NodeHandle nh;

            ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/" + robot_namespace + "/scan", 10);
            lidar_pubs[r] = lidar_pub;


            // sensor_msgs::LaserScan scan_msg;

            // scan_msg.header.frame_id = lidar_frame_id;
            scan_msg.angle_min = -fov;
            scan_msg.angle_max = fov;
            scan_msg.angle_increment = fov / num_beams;
            scan_msg.time_increment = 0.001;
            scan_msg.scan_time = 0.1;
            scan_msg.range_min = 0.0;
            scan_msg.range_max = max_range;
            scan_msg.ranges = lidars[r]->ranges;
            scan_msg.header.stamp = ros::Time::now();

            scan_msgs[r] = scan_msg;


            // tf::TransformBroadcaster scan_broadcaster;
            // geometry_msgs::Quaternion scan_quat = tf::createQuaternionMsgFromYaw(lidarinitialTheta);

            // // geometry_msgs::TransformStamped scan_trans;
            // scan_trans.header.stamp = ros::Time::now();
            // // scan_trans.header.frame_id = robot_frame_id;
            // scan_trans.child_frame_id = lidar_frame_id;

            // scan_trans.transform.translation.x = lidarinitialX;
            // scan_trans.transform.translation.y = lidarinitialY;
            // // scan_trans.transform.translation.z = 0.0;
            // scan_trans.transform.rotation = scan_quat;

            // //send the transform
            // scan_broadcaster.sendTransform(scan_trans);

            // static tf2_ros::TransformBroadcaster br;
            // geometry_msgs::TransformStamped transform_stamped;

            // transform_stamped.header.frame_id = parentFrameID;
            // transform_stamped.child_frame_id = lidar_frame_id;
            // transform_stamped.header.stamp = ros::Time::now();

            // transform_stamped.transform.translation.x = 0.0;
            // transform_stamped.transform.translation.y = 0.0;
            // transform_stamped.transform.translation.z = 0.0;

            // tf2::Quaternion q;
            // q.setRPY(0, 0, 0);
            // q.normalize();

            // transform_stamped.transform.rotation.x = q.x();
            // transform_stamped.transform.rotation.y = q.y();
            // transform_stamped.transform.rotation.z = q.z();
            // transform_stamped.transform.rotation.w = q.w();

            // br.sendTransform(transform_stamped);

            r++;

        }

    }

    
    ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz


    std::string map_path = jsonData["map"].asString();
    world.loadFromImage("/home/loris/Desktop/university/master/rp/git_project/robot_programming_project/ros_ws/src/my_package/test_data/" + map_path); // to load the map image


    world.draw();
    cv::waitKey(1);


    while (ros::ok())
    {
        ros::spinOnce();


        for (int i = 0; i < k; i++) {
            robots[i]->tv = vels[0];
            robots[i]->rv = vels[1];
        }



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


        for (int i = 0; i < r; i++) {

            scan_msgs[i].header.stamp = ros::Time::now();
            scan_msgs[i].ranges = lidars[i]->ranges; 

            lidar_pubs[i].publish(scan_msgs[i]);
    

        }


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




