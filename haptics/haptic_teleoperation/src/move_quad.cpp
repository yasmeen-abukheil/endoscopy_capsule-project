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

//const double M_PI=3.14159265359 ;
const double deg_to_rad = M_PI / 180.0 ;
class move_quadrotor
{
public:
    move_quadrotor(ros::NodeHandle & n_) : n(n_)
    {
        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    }

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

    void turn(double val)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.2;
        msg.linear.y =  0.0 ;
        msg.linear.z =  0.0;
        msg.angular.x = 0.0 ;
        msg.angular.y = 0.0 ;
        msg.angular.z = val ;
        vel_pub.publish(msg);
    }


    void init_move(double val)
    {
        for ( int i = 0 ; i < 1000000 ; i++ )
        {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.linear.y =  0.0 ;
            msg.linear.z =  val;
            msg.angular.x = 0.0 ;
            msg.angular.y = 0.0 ;
            msg.angular.z = 0.0 ;
            vel_pub.publish(msg);

        }
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y =  0.0 ;
        msg.linear.z =  0.0;
        msg.angular.x = 0.0 ;
        msg.angular.y = 0.0 ;
        msg.angular.z = 0.0 ;
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

    ros::init(argc, argv, "move_quad");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(150);

    // create an object
    move_quadrotor move_robot_obj(n);

    move_robot_obj.init_move(0.5) ;
sleep(50) ; 
    move_robot_obj.init_move(0.0) ;
bool flag = true ;

    while(ros::ok())
    {


     /*   if (flag)
        {
        for (int i=0 ;i <99999; i++)
        {
            std::cout << "turn left" << std::endl ;

            move_robot_obj.turn(-0.785);
        }
        flag = false ;
        }
        loop_rate.sleep();

        for (int i=0 ;i <99999; i++)
        {
            std::cout << "move forward" << std::endl ;
            move_robot_obj.move(1.0);
        }

        loop_rate.sleep();
        for (int i=0 ;i <99999; i++)
        {
            std::cout << "turn right" << std::endl ;

            move_robot_obj.turn(1.75);
        }
        loop_rate.sleep();

        for (int i=0 ;i <99999; i++)
        {
            std::cout << "move backward" << std::endl ;

            move_robot_obj.move(1.0);
        }
        loop_rate.sleep();


        for (int i=0 ;i <99999; i++)
        {
            std::cout << "turn right" << std::endl ;

            move_robot_obj.turn(-1.2);
        }*/
        loop_rate.sleep();



    }
    return 0;
}


