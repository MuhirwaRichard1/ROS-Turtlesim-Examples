#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>


void sendMessageReceive(const turtlesim::Pose &msg){
    ROS_INFO_STREAM (std::setprecision(2)<<std::fixed <<
    "position=("<< msg.x << ","<< msg.y << ")" << "direction=" <<msg.theta);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub= nh.subscribe("turtle1/pose", 100, &sendMessageReceive);

    // let ROS take overSS
    ros::spin();
}