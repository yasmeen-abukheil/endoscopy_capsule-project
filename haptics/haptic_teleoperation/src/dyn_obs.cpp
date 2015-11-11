#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <phantom_omni/OmniFeedback.h>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/TwistArray.h>
#include <haptic_teleoperation/ContourData.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <haptic_teleoperation/potential_fieldConfig.h>
#include <phantom_omni/PhantomButtonEvent.h>
#include <gazebo_msgs/ModelState.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

bool flag = true ;
class move_robot
{
public:
    move_robot(ros::NodeHandle & n_) : n(n_)
    {
        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
    };


    void move(double val)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = val;
        msg.linear.y =  0.0 ;
        msg.linear.z =  0.0;
        msg.angular.x = 0 ;
        msg.angular.y = 0 ;
        msg.angular.z = 0 ;
        vel_pub.publish(msg);
    }
private:
    // ROS
    ros::NodeHandle n;
    ros::Publisher vel_pub ;

} ;


int main(int argc, char **argv)
{

    // ROS node
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(50);

    // create an object
    move_robot move_robot_obj_1(n);

    while(ros::ok())
    {

            for (int i=0 ;i <999999; i++)
            {
                std::cout << "move forward" << std::endl ;
                  move_robot_obj_1.move(1.0);
            }

            for (int i=0 ;i <999999; i++)
            {
                std::cout << "turn right" << std::endl ;
                move_robot_obj_1.move(-1.0 );
            }
            loop_rate.sleep();
    }

    return 0;
}

