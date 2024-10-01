#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <iomanip>
#include <sstream>

double xr = 0.0;
double yr = 0.0;
double theta = 0.0;
double xg = 0.0;
double yg = 0.0;
double KPpos = 0.1;
double KPh = 0.01;
const double Dh = 0.1;
const double D_pos = 0.1;

void goToPosition(double a, double b, ros::Publisher& pub, ros::Rate& rate) {
    xg = a;
    yg = b;
    
    geometry_msgs::Twist vel_msg;
    
    while (ros::ok()) {
        double dx = xg - xr;
        double dy = yg - yr;
        
        double e_pos = std::sqrt(dx*dx + dy*dy);
        double e_h = std::atan2(dy, dx) - theta;
        
        // Normalize e_h to be between -pi and pi
        e_h = std::fmod(e_h + M_PI, 2 * M_PI) - M_PI;
        
        if (std::abs(e_h) > Dh) {
            vel_msg.linear.x = 0;
            vel_msg.angular.z = KPh * e_h;
        } else {
            vel_msg.linear.x = KPpos * e_pos;
            vel_msg.angular.z = 0;
        }
        
        pub.publish(vel_msg);
        
        if (e_pos <= D_pos) {
            break;  // Exit the loop if we've reached the goal
        }
        
        ros::spinOnce();
        if (!rate.sleep()) {
            ROS_WARN("Loop exceeded desired rate");
        }
    }
    
    // Stop the robot
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    std::stringstream ss;
    ss << std::setprecision(2) << std::fixed
       << "position=(" << msg->x << "," << msg->y << ")"
       << " direction=" << msg->theta;
    ROS_INFO_STREAM(ss.str());
    
    xr = msg->x;
    yr = msg->y;
    theta = msg->theta;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "divide_conq");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 100, poseCallback);
    
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    
    ros::Rate rate(100);  // Increased rate for smoother control
    
    // Wait for services
    teleportClient.waitForExistence();
    penClient.waitForExistence();
    clearClient.waitForExistence();
    
    while (ros::ok()) {
        // Example usage
        goToPosition(5.0, 5.0, pub, rate);
        
        // You can add more goal positions here if needed
         goToPosition(2.0, 2.0, pub, rate);
         goToPosition(8.0, 8.0, pub, rate);
        
        // Check for shutdown between goals
        if (!ros::ok()) break;
        
        // Optional: Wait a bit before starting the next goal
        ros::Duration(0.01).sleep();
    }
    
    return 0;
}