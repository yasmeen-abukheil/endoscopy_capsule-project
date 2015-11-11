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



#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <sstream>
#include <gazebo_msgs/SetModelState.h>

//const double M_PI=3.14159265359 ;
const double deg_to_rad = M_PI / 180.0 ;
class move_tube
{
public:
    move_tube(ros::NodeHandle & n_) : n(n_)
    {
        model_state_pub = n_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    };

    void move( int x, int y , int z  )
    {
        geometry_msgs::Pose start_pose;
        start_pose.position.x = x;
        start_pose.position.y = y;
        start_pose.position.z = z;
        start_pose.orientation.x = 0;
        start_pose.orientation.y =0 ;
        start_pose.orientation.z =0 ;
        start_pose.orientation.w = 0;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = "camera";
        modelstate.reference_frame = (std::string) "world";
        modelstate.pose = start_pose;
        //    modelstate.twist = start_twist;

        model_state_pub.publish(modelstate);

    }

private:
    // ROS
    ros::NodeHandle n;
    ros::Publisher model_state_pub ;
} ;


int main(int argc, char **argv)
{

    // ROS node
    ros::init(argc, argv, "move_quad");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(150);

    // create an object
    move_tube move_robot_obj(n);


    while(ros::ok())
    {
        move_robot_obj.move(0, 0 , 0 );
        sleep(2);


    }
    return 0;
}


