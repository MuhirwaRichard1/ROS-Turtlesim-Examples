#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <signal.h>

double xr = 0.0;
double yr = 0.0;
double theta = 0.0;

double xg = 8.0;
double yg = 9.0;
double theta_g = 0.0;

double dx = 0.0;
double dy = 0.0;

double v = 0.0;
double w = 0.0;

double KPpos = 0.2;      //5.5
double KPh = 0.89;    //6.95
double KP_theta = 0.01;  //3.14

double D_pos = 0.001;
double Dh = 0.001;

double e_pos = 0.0;
double e_h = 0.0;
double e_theta = 0.0;

bool keep_running = true;  // This flag will control when to stop the loop

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    ROS_INFO("Interrupt signal received. Shutting down...");
    keep_running = false;  // Set the flag to false to break the loop
}

void poseMessages(const turtlesim::Pose &Smsg) {
    ROS_INFO_STREAM(std::setprecision(10) << std::fixed <<
    "position=(" << Smsg.x << "," << Smsg.y << ")" << " direction=" << Smsg.theta);
    // Assign values from turtle bot to x1, y1, and theta1
    xr = Smsg.x;
    yr = Smsg.y;
    theta = Smsg.theta;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "divide_conq");
    ros::NodeHandle nh;

    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 100, &poseMessages);

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
    penArgument.request.width = 1;
    penArgument.request.off = 1;
    penClient.call(penArgument);

    // Teleport the turtle to the starting position (1, 1) with heading 0
    teleportArgument.request.x = 2.0;
    teleportArgument.request.y = 1.0;
    teleportArgument.request.theta = 3.14;
    teleportClient.call(teleportArgument);

    penArgument.request.off = 0;
    penClient.call(penArgument);

    ros::Rate rate(2);

    // Main loop, controlled by `keep_running`
    while (ros::ok() && keep_running) {
        geometry_msgs::Twist msg;

        do {
            // Compute the current position of the robot: (xr, yr)
            dx = xg - xr;
            dy = yg - yr;

            // Compute the position and heading errors (e_pos, e_h)
            e_pos = sqrt(dx * dx + dy * dy);
            e_h = atan2(dy, dx) - theta;
            e_theta = theta_g - atan2(dy, dx);

            // Normalize heading and orientation errors to [-pi, pi]
            e_h = atan2(sin(e_h), cos(e_h));
            e_theta = atan2(sin(e_theta), cos(e_theta));

            // Set velocities
            v = KPpos * e_pos;
            w = KPh * e_h + KP_theta * e_theta;

            // Velocity saturation to avoid excessive speed
            //double v_max = 2.0;  // Max linear velocity
            //double w_max = 2.0;  // Max angular velocity
            //v = std::max(-v_max, std::min(v_max, v));
            //w = std::max(-w_max, std::min(w_max, w));

            // Send velocities (v, w) to robot
            msg.linear.x = v;
            msg.angular.z = w;

            // Publish on geometry/Twist topic
            pub.publish(msg);

            // Pause for some time
            rate.sleep();

            ros::spinOnce();
        } while (fabs(e_pos > D_pos)  && keep_running);  // Ensure the loop stops if `keep_running` becomes false

        // Send velocities (0,0) to the robot
        msg.linear.x = 0;
        msg.angular.z = 0;
        pub.publish(msg);
        ros::Duration(1.0).sleep();  // Pause for 1 second after reaching each position
        // Teleport the turtle to the goal location (optional)
        teleportArgument.request.x = 8.0;
        teleportArgument.request.y = 9.0;
        teleportArgument.request.theta = 0.0;
        teleportClient.call(teleportArgument);
    }

    return 0;
}
