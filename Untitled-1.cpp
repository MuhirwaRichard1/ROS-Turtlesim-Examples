#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <stdlib.h>


int main (int argc, char **argv){
    ros::init(argc, argv, "Publish_velocity");
    ros::NodeHandle nh;
    ros::Publisher pub= nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    ros::Rate rate(2);

    while(ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = 3;
        msg.angular.z =0.5;
        pub.publish(msg);
        ROS_INFO_STREAM("Linear velocity: "<<msg.linear.x <<"Angular velocity: "<<msg.angular.z);
        rate.sleep();
    }


}