#include <ros/ros.h>
#include <std_msgs/String.h>

void callBackMessage(const std_msgs::String::ConstPtr &msg){
    ROS_INFO_STREAM("I heard  " << msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub= nh.subscribe("chatter", 100, &callBackMessage);

    ros::spin();
    return 0;
}