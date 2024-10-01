#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>


int main(int argc, char **argv){
    bool success = true;

    ros::init(argc, argv, "draw_rpp");
    ros::NodeHandle nh;

    ros::service::waitForService("spawn");
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

    turtlesim::Spawn             spawnArgument;

    spawnArgument.request.x =4;  
    spawnArgument.request.y =1; 
    spawnArgument.request.theta = 3.14159 / 2;
    spawnArgument.request.name = "turtle2" ;
    spawnClient.call(spawnArgument);

    spawnArgument.request.x =7;  
    spawnArgument.request.y =1; 
    spawnArgument.request.theta = 3.14159 / 2;
    spawnArgument.request.name = "turtle3" ;
    spawnClient.call(spawnArgument);

    // create objects for services required
    ros::service::waitForService("turtle1/teleport_absolute");
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    ros::service::waitForService("turtle1/set_pen");
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

    ros::service::waitForService("turtle2/teleport_absolute");
    ros::ServiceClient teleportClient_2 = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle2/teleport_absolute");

    ros::service::waitForService("turtle2/set_pen");
    ros::ServiceClient penClient_2 = nh.serviceClient<turtlesim::SetPen>("turtle2/set_pen");

    ros::service::waitForService("turtle3/teleport_absolute");
    ros::ServiceClient teleportClient_3 = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle3/teleport_absolute");

    ros::service::waitForService("turtle3/set_pen");
    ros::ServiceClient penClient_3 = nh.serviceClient<turtlesim::SetPen>("turtle3/set_pen");


    ros::service::waitForService("clear");
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");

    // initialize objects to service the services
    turtlesim::TeleportAbsolute  teleportArgument;
    turtlesim::SetPen            penArgument;

    std_srvs::Empty              clearArgument;

    success = clearClient.call(clearArgument);

                /*R*/
    // teleport to (1,1) OFF     
    penArgument.request.off = 1;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 1;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (1,9) ON     
    penArgument.request.off = 0;
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 10;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 1; 
    teleportArgument.request.y = 9;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (3, 9) ON     
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 3;
    teleportArgument.request.y = 9;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (3, 5) ON     
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 3;
    teleportArgument.request.y = 5;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (1, 5) ON     
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 1;
    teleportArgument.request.y = 5;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (3,1) ON     
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 3;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

        // teleport to (1,1) OFF     
    penArgument.request.off = 1;
    success = penClient.call(penArgument);

    teleportArgument.request.x = 1;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    success = teleportClient.call(teleportArgument);

                /* P */

        // teleport to (4, 1) OFF   

    penArgument.request.off = 1;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 4;
    teleportArgument.request.y = 1;
    success = teleportClient_2.call(teleportArgument);

        // teleport to (4, 9) ON     
    penArgument.request.off = 0;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 4;
    teleportArgument.request.y = 9;
    success = teleportClient_2.call(teleportArgument);

        // teleport to (6, 9) ON     
    penArgument.request.off = 0;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 6;
    teleportArgument.request.y = 9;
    success = teleportClient_2.call(teleportArgument);

        // teleport to (6, 5) ON     
    penArgument.request.off = 0;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 6;
    teleportArgument.request.y = 5;
    success = teleportClient_2.call(teleportArgument);

        // teleport to (4, 5) ON     
    penArgument.request.off = 0;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 4;
    teleportArgument.request.y = 5;
    success = teleportClient_2.call(teleportArgument);

        // teleport to (4, 1) OFF    
    penArgument.request.off = 1;
    success = penClient_2.call(penArgument);

    teleportArgument.request.x = 4;
    teleportArgument.request.y = 1;
    success = teleportClient_2.call(teleportArgument);

                /* P */

        // teleport to (7,1) OFF     
    penArgument.request.off = 1;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 7;
    teleportArgument.request.y = 1;
    success = teleportClient_3.call(teleportArgument);

        // teleport to (7, 9) ON     
    penArgument.request.off = 0;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 7;
    teleportArgument.request.y = 9;
    success = teleportClient_3.call(teleportArgument);

        // teleport to (9, 9) ON     
    penArgument.request.off = 0;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 9;
    teleportArgument.request.y = 9;
    success = teleportClient_3.call(teleportArgument);

        // teleport to (9, 5) ON     
    penArgument.request.off = 0;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 9;
    teleportArgument.request.y = 5;
    success = teleportClient_3.call(teleportArgument);

        // teleport to (7, 5) ON     
    penArgument.request.off = 0;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 7;
    teleportArgument.request.y = 5;
    success = teleportClient_3.call(teleportArgument);

        // teleport to (7, 1) OFF     
    penArgument.request.off = 1;
    success = penClient_3.call(penArgument);

    teleportArgument.request.x = 7;
    teleportArgument.request.y = 1;
    success = teleportClient_3.call(teleportArgument);
}