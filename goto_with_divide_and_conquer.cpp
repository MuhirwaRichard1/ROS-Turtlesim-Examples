#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <iomanip>

// Global variables for current robot pose
double xr = 0.0;
double yr = 0.0;
double theta = 0.0;

// Control constants
const double KPpos = 5; //0.1
const double KPh = 5; // 0.5 
const double Dpos = 0.0001;
const double Dh = 0.0005;

// Callback to update robot's current position
void poseMessages(const turtlesim::Pose::ConstPtr& Smsg) {
    ROS_INFO_STREAM (std::setprecision(2)<<std::fixed <<
    "position=("<< Smsg->x << ","<< Smsg->y << ")" << "direction=" <<Smsg->theta);
    xr = Smsg->x;
    yr = Smsg->y;
    theta = Smsg->theta;
}

// Function to move the turtlebot to the target (xg, yg)
void gotoPosition(double xg, double yg, ros::Publisher& pub, ros::Rate& rate) {
    geometry_msgs::Twist msg;
    double dx, dy, e_pos, e_h;

    do {
        // Compute the distances to the target
        dx = xg - xr;
        dy = yg - yr;

        // Compute position error (Euclidean distance) and heading error
        e_pos = sqrt(dx * dx + dy * dy);
        e_h = atan2(dy, dx) - theta;

        // Normalize heading error between -pi and pi
        if (e_h > M_PI)
            e_h -= 2 * M_PI;
        else if (e_h < -M_PI)
            e_h += 2 * M_PI;

        // Control logic
        if (fabs(e_h) > Dh) {
            // Rotate in place to correct heading
            msg.linear.x = 0.0;   // Stop forward movement
            msg.angular.z = KPh * e_h; // Correct heading
        } else {
            // Move forward towards the target
            msg.linear.x = KPpos * e_pos; // Move forward
            msg.angular.z = 0.0;          // No rotation
        }

        // Publish the velocity message
        pub.publish(msg);

        // Pause for a short time
        rate.sleep();

        ros::spinOnce(); // Update the position

    } while (e_pos > Dpos); // Continue until close enough to the goal

    // Stop the robot when target is reached
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);

    ros::Duration(1.0).sleep();  // Pause for 1 second after reaching each position
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "move_to_positions");
    ros::NodeHandle nh;

    // Publisher to control turtlebot velocity
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);

    // Subscriber to get the turtlebot's current pose
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 100, poseMessages);

    // Wait for teleport service and pen service to be available
    ros::service::waitForService("turtle1/teleport_absolute");
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    ros::service::waitForService("turtle1/set_pen");
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

    ros::service::waitForService("clear");
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");

    // Create service objects for the services
    turtlesim::TeleportAbsolute teleportArgument;
    turtlesim::SetPen penArgument;
    std_srvs::Empty clearArgument;

    // Clear the screen and reset turtle state
    clearClient.call(clearArgument);

    // Set the pen on (you can change the color using the r, g, b values)
    penArgument.request.r = 255;
    penArgument.request.g = 255;
    penArgument.request.b = 255;
    penArgument.request.width = 2;
    penArgument.request.off = 1;
    penClient.call(penArgument);

    // Teleport the turtle to the starting position (1, 1) with heading 0
    teleportArgument.request.x = 1.0;
    teleportArgument.request.y = 1.0;
    teleportArgument.request.theta = 0.0;
    teleportClient.call(teleportArgument);

    ros::Rate rate(10); // Loop rate for controlling the turtle

    // Move the turtle to the sequence of positions
    gotoPosition(1.0, 1.0, pub, rate);
    penArgument.request.off = 0;
    penClient.call(penArgument);
    //gotoPosition(1.0, 9.0, pub, rate);
    gotoPosition(1.0, 5.5, pub, rate);
    gotoPosition(3.5, 9.5, pub, rate);
    gotoPosition(6.0, 5.5, pub, rate);
    gotoPosition(6.0, 1.0, pub, rate);
    gotoPosition(1.0, 1.0, pub, rate);
    penArgument.request.off = 1;
    penClient.call(penArgument);
    gotoPosition(6.0, 5.5, pub, rate);
    penArgument.request.off = 0;
    penClient.call(penArgument);
    gotoPosition(1.0, 5.5, pub, rate);
    penArgument.request.off = 1;
    penClient.call(penArgument);
    gotoPosition(1.5, 4.0, pub, rate);
    penArgument.request.off = 0;
    penClient.call(penArgument);
    gotoPosition(1.5, 3.0, pub, rate);
    gotoPosition(2.5, 3.0, pub, rate);
    gotoPosition(2.5, 4.0, pub, rate);
    gotoPosition(1.5, 4.0, pub, rate);
    penArgument.request.off = 1;
    penClient.call(penArgument);
    gotoPosition(4.0, 3.5, pub, rate);
    penArgument.request.off = 0;
    penClient.call(penArgument);
    gotoPosition(5.0, 3.5, pub, rate);
    gotoPosition(5.0, 1.0, pub, rate);
    gotoPosition(4.0, 1.0, pub, rate);
    gotoPosition(4.0, 3.5, pub, rate);
    penArgument.request.off = 1;
    penClient.call(penArgument);
    gotoPosition(1.0, 1.0, pub, rate);

    ros::spin();  // Keep the node alive to process callback functions

    return 0;
}
