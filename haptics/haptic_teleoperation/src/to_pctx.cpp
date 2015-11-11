#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "pctx_control/Control.h"
#include "std_msgs/String.h"
#include "pctx_control/pctx.h"
#include <sstream>
#include "std_msgs/String.h"
#include "pctx_control/pctx.h"

ros::Publisher pub;
pctx_control::Control controlmsg;



void function_callback (const geometry_msgs::Twist& msg)
{
    std::cout << "in the call back function" << std::endl ;
    controlmsg.channel = 0 ;
    std::cout << "filled the channel" << std::endl ;;
    controlmsg.values.push_back(msg.linear.x);
    controlmsg.values.push_back(msg.linear.y);
    controlmsg.values.push_back(msg.linear.z);
    controlmsg.values.push_back(0.0);
    controlmsg.values.push_back(0.0);
    controlmsg.values.push_back(0.0);
    controlmsg.values.push_back(0.0);
    controlmsg.values.push_back(0.0);
    controlmsg.values.push_back(0.0);
    pub.publish(controlmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle n;
    pub = n.advertise<pctx_control::Control>("sendPCTXControl", 1000);
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, function_callback);
    // Handle ROS callbacks until shutdow
    ros::spin();
    return 0;
}

