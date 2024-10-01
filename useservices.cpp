#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv){

    bool success = true;

    //initialize ros and create node
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nh;

    // create client object for required services
    ros::service::waitForService("turtle1/teleport_absolute");
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    ros::service::waitForService("turtle1/set_pen");
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

    ros::service::waitForService("clear");
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");

    //create service objects for services 
    turtlesim::TeleportAbsolute teleportArgument;
    turtlesim::SetPen           penArgument;
    std_srvs::Empty             clearArgument;

    //reset the turtle 
    success = clearClient.call(clearArgument);
    if(!success){
        ROS_ERROR_STREAM("failed to clear the path");
    }
    //set pen for turtle bot
   /* penArgument.request.off = 1;
    success = penClient.call(penArgument);
    if(!success){
        ROS_ERROR_STREAM("failed to set the pen");
    }
    */

        //set the pen and vary the color
    penArgument.request.off = 0;
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 0;
    penArgument.request.width = 10; //bigger line
    success = penClient.call(penArgument);
    //teleport the turtle
    teleportArgument.request.x = 2;
    teleportArgument.request.y =3;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set the pen and vary the color
    penArgument.request.off = 1;
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 0;
    penArgument.request.width = 10; //bigger line

    success = penClient.call(penArgument);
    if(!success){
        ROS_ERROR_STREAM("failed to change the color of the pen");
    }
}