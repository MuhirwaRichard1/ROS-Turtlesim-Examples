#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "Hello_publish");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate rate(10);

    int counter = 0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream letter;
        letter<<"Hello world" << counter;

        msg.data = letter.str();

        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        counter++;
    }
    return 0;
}