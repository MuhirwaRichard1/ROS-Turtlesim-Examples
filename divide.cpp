#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>

/*
*/

double xr = 0.0;
double yr = 0.0;
double theta = 0.0;
double xg = 8.0;
double yg = 8.0;
double dx = 0.0;
double dy = 0.0;

double v = 0.0;
double w = 0.0;

double KPpos = 0.50;
double KPh = 0.50;

double D_pos = 0.001;
double Dh = 0.001;

double e_pos = 0.0;
double e_h = 0.0;
/*void goto(double a, double b){
...;
}
*/
double xr1 = 0.0;
double yr1 = 0.0;
double theta1 = 0.0;

double xg1 = 8.0;
double yg1 = 8.0;
double theta_g1 = 0.0;

double dx1 = 0.0;
double dy1 = 0.0;

double v1 = 0.0;
double w1 = 0.0;

double KPpos1 = 0.2;      //5.5
double KPh1 = 0.89;       //6.95
double KP_theta1 = 0.01;  //3.14

double D_pos1 = 0.001;
double Dh1 = 0.001;

double e_pos1 = 0.0;
double e_h1 = 0.0;
double e_theta1 = 0.0;

bool keep_running = true;  // This flag will control when to stop the loop

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    ROS_INFO("Interrupt signal received. Shutting down...");
    keep_running = false;  // Set the flag to false to break the loop
}

void poseMessages(const turtlesim::Pose &Smsg){
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<
    "position=("<< Smsg.x << ","<< Smsg.y << ")" << "direction=" <<Smsg.theta);
    // Assign values from turtle bot to x1, y1, and theta1
    xr = Smsg.x;
    yr = Smsg.y;
    theta = Smsg.theta;
}

void poseMessages1(const turtlesim::Pose &Smsg1){
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed <<
    "position=("<< Smsg1.x << ","<< Smsg1.y << ")" << "direction=" <<Smsg1.theta);
    // Assign values from turtle bot to x1, y1, and theta1
    xr1 = Smsg1.x;
    yr1 = Smsg1.y;
    theta1 = Smsg1.theta;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "divide_conq");
    ros::NodeHandle nh;

    //Services for spawning
    ros::service::waitForService("spawn");
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

    turtlesim::Spawn             spawnArgument;

    spawnArgument.request.x =2.0;  
    spawnArgument.request.y =1.0; 
    spawnArgument.request.theta = 3.14159 / 2;
    spawnArgument.request.name = "turtle2" ;
    spawnClient.call(spawnArgument);


    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 100, &poseMessages);

    // Publishing to second turtle
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 100);
    ros::Subscriber sub1 = nh.subscribe("turtle2/pose", 100, &poseMessages1);

    ros::service::waitForService("turtle1/teleport_absolute");
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    ros::service::waitForService("turtle1/set_pen");
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

    //Ros service call for Turtle_2 
    ros::service::waitForService("turtle2/teleport_absolute");
    ros::ServiceClient teleportClient_2 = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle2/teleport_absolute");

    ros::service::waitForService("turtle2/set_pen");
    ros::ServiceClient penClient_2 = nh.serviceClient<turtlesim::SetPen>("turtle2/set_pen");

    //Ros service call for clear service works for all turtles
    ros::service::waitForService("clear");
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");

    //create service objects for the services
    turtlesim::TeleportAbsolute     teleportArgument;
    turtlesim::SetPen               penArgument;
    std_srvs::Empty                 clearArgument;

        // Clear the screen and reset turtle state
    clearClient.call(clearArgument);

    // Set the pen on (you can change the color using the r, g, b values)
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 2;
    penArgument.request.off = 1;
    penClient.call(penArgument);

    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 2;
    penArgument.request.off = 1;
    penClient_2.call(penArgument);

    // Teleport the turtle to the starting position (1, 1) with heading 0
    teleportArgument.request.x = 2.0;
    teleportArgument.request.y = 1.0;
    teleportArgument.request.theta = 180.0;
    teleportClient.call(teleportArgument);

    teleportArgument.request.x = 2.0;
    teleportArgument.request.y = 1.0;
    teleportArgument.request.theta = 3.14;
    teleportClient_2.call(teleportArgument);
    

    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 2;
    penArgument.request.off = 0;
    penClient.call(penArgument);

    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 2;
    penArgument.request.off = 0;
    penClient_2.call(penArgument);

    ros::Rate rate(2);


    while(ros::ok() && keep_running){
        geometry_msgs::Twist msg;
        geometry_msgs::Twist msg1;
    do{
        
        //compute the current position of the robot : (xr, yr)
        /* We have to subscribe to cmd_vel topic in order to get the current position of the robot */
        /*Then assign the values of cmd_vel to xr and yr */
       
        //Compute the distance from robot position to the target (dx, dy)
        dx = xg - xr;
        dy = yg - yr;
        // Compute the current position of the robot: (xr, yr)
        dx1 = xg1 - xr1;
        dy1 = yg1 - yr1;

        //Compute the position of heading errors (e_pos, e_h)
        e_pos = sqrt(dx*dx + dy*dy);
        e_h   = atan2 (dy, dx) - (theta);

        // Compute the position and heading errors (e_pos, e_h)
        e_pos1 = sqrt(dx1 * dx1 + dy1 * dy1);
        e_h1 = atan2(dy1, dx1) - theta1;
        e_theta1 = theta_g1 - atan2(dy1, dx1);

         if (e_h > Dh){ /*Dh is going to be a small constant number*/
               v = 0;
               w = KPh * e_h;
              }

         else {
                // set forward velocity :
                v = KPpos * e_pos;
                // set angular velocity :
                w = 0;
                }

        // Set velocities
        v1 = KPpos1 * e_pos1;
        w1 = KPh1 * e_h1 + KP_theta1 * e_theta1;

        // send velociteies (v, w) to robot  for MIMO
        msg1.linear.x =  v1;
        msg1.angular.z = w1;

        // Send velocities (v, w) to robot
        msg.linear.x = v;
        msg.angular.z = w;

        /* Publish on geometry/Twist topic */
        pub.publish(msg);
        pub1.publish(msg1);
        // Pause for some time
        /* call sleep function with rate */
        rate.sleep();

  ros::spinOnce();

    } while(e_pos > D_pos && keep_running); //while , /*D_pos is a constant small number
             // Send velocities(0,0) to the robot
             msg.linear.x  = 0;
             msg.angular.z = 0;
             pub.publish(msg);

             msg1.linear.x  = 0;
             msg1.angular.z = 0;
             pub1.publish(msg1);

    }

    return 0;
}