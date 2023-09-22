#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include <thread>
#include <chrono>

using namespace std;

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the cmd_vel topic
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000);

    // Create a Twist message with desired linear and angular velocities
    geometry_msgs::Twist cmd_vel_msg;

    ros::Rate loop_rate(10);  // Publish at a rate of 10 Hz

    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (ros::ok())
    {
        
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;

        // sleep(0.025);
        this_thread::sleep_for(chrono::milliseconds(25)); // sleep for x milliseconds
        cmd_vel_pub.publish(cmd_vel_msg);  // Publish the Twist message
        ros::spinOnce();
    
        char ch = getchar();

        if (ch == 27) {
        char next_ch = getchar(); // get the char after 'ESC'

            if(next_ch == '[') {
                ch = getchar(); // get the character after '['
                switch (ch) {
                case 'A': cout << "up\n"; cmd_vel_msg.linear.x = 2.2; cout.flush(); break;
                case 'B': cout << "down\n"; cmd_vel_msg.linear.x = -2.2; cout.flush(); break;
                case 'C': cout << "right\n"; cmd_vel_msg.angular.z = -1.8; cout.flush(); break;
                case 'D': cout << "left\n"; cmd_vel_msg.angular.z = 1.8; cout.flush(); break;
                default: cerr << "Invalid command: " << ch << endl; cout.flush(); break;
                }
            }
            else break;
        } 


        cmd_vel_pub.publish(cmd_vel_msg);  // Publish the Twist message
        ros::spinOnce();
    }


    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // to reset the terminal


    return 0;
}