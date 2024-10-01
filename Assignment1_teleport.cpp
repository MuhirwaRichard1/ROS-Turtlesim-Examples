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

            /*PART 1*/

    //set pen for turtle bot (1,1) OFF
    penArgument.request.off = 1;
    success = penClient.call(penArgument);
    if(!success){
        ROS_ERROR_STREAM("failed to set the pen");
    }
    

    //teleport the turtle (1,1)
    teleportArgument.request.x = 1;
    teleportArgument.request.y =1;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set the pen and vary the color (1,5.5) ON
    penArgument.request.off = 0;
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 1; //bigger line

    success = penClient.call(penArgument);
    if(!success){
        ROS_ERROR_STREAM("failed to change the color of the pen");
    }

//teleport the turtle (1,5.5)
    teleportArgument.request.x = 1;
    teleportArgument.request.y =5.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (3.5, 9.5) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    //teleport the turtle (3.5, 9.5)

    teleportArgument.request.x = 3.5;
    teleportArgument.request.y =9.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (6,5.5) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

    //teleport the turtle (6 ,5.5)

    teleportArgument.request.x = 6;
    teleportArgument.request.y = 5.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (6,1) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (6,1)
        teleportArgument.request.x = 6;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (1,1) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (1,1)
        teleportArgument.request.x = 1;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

            /*PART 2*/

    //set pen for turtle bot (6,5.5) OFF
    penArgument.request.off = 1;
    success = penClient.call(penArgument);

//teleport the turtle (6,5.5)
        teleportArgument.request.x = 6;
    teleportArgument.request.y = 5.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (1,5.5) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (1,5.5)
        teleportArgument.request.x = 1;
    teleportArgument.request.y =5.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (1.5,4) OFF
    penArgument.request.off = 1;
    success = penClient.call(penArgument);

//teleport the turtle (1.5,4)
        teleportArgument.request.x = 1.5;
    teleportArgument.request.y = 4;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (1.5, 3) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (1.5, 3)
        teleportArgument.request.x = 1.5;
    teleportArgument.request.y = 3;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (2.5, 3) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (2.5, 3)
        teleportArgument.request.x = 2.5;
    teleportArgument.request.y = 3;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (2.5, 4) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (2.5, 4)
        teleportArgument.request.x = 2.5;
    teleportArgument.request.y = 4;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (1.5, 4) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (1.5, 4)
        teleportArgument.request.x = 1.5;
    teleportArgument.request.y = 4;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    /* PART 3*/

    //set pen for turtle bot (4, 3.5) OFF
    penArgument.request.off = 1;
    success = penClient.call(penArgument);

//teleport the turtle (4, 3.5)
        teleportArgument.request.x = 4;
    teleportArgument.request.y = 3.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (5, 3.5) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (5, 3.5)
        teleportArgument.request.x = 5;
    teleportArgument.request.y = 3.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

    //set pen for turtle bot (5,1) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (5,1)
        teleportArgument.request.x = 5;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

        //set pen for turtle bot (4,1) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (4,1)
        teleportArgument.request.x = 4;
    teleportArgument.request.y = 1;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }

        //set pen for turtle bot (4, 3.5) ON
    penArgument.request.off = 0;
    success = penClient.call(penArgument);

//teleport the turtle (4, 3.5)
        teleportArgument.request.x = 4;
    teleportArgument.request.y = 3.5;
    teleportArgument.request.theta = 3.14159 / 2;
    
    success = teleportClient.call(teleportArgument);
    if(!success){
        ROS_ERROR_STREAM("turtle faild to teleport");
    }


}