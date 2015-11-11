////#include "ros/ros.h"
////#include "std_msgs/String.h"
////#include "nav_msgs/Odometry.h"
////#include "sensor_msgs/PointCloud.h"
////#include <visualization_msgs/Marker.h>
////#include <geometry_msgs/Twist.h>
////#include <geometry_msgs/Pose.h>

////#include <visualization_msgs/MarkerArray.h>
////#include <cmath>
////#include <phantom_omni/OmniFeedback.h>
////#include <dynamic_reconfigure/server.h>
////#include <haptic_teleoperation/TwistArray.h>
////#include <haptic_teleoperation/ContourData.h>
////#include <laser_geometry/laser_geometry.h>
////#include <tf/transform_listener.h>
////#include <haptic_teleoperation/potential_fieldConfig.h>
////#include <phantom_omni/PhantomButtonEvent.h>
////#include <gazebo_msgs/ModelState.h>
////#include <Eigen/Eigen>
////#include <Eigen/Geometry>

//////const double M_PI=3.14159265359 ;
////const double deg_to_rad = M_PI / 180.0 ;
////bool flag = true ;
////class move_robot
////{
////public:
////    move_robot(ros::NodeHandle & n_) : n(n_)
////    {
////        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
////        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

////    };


////    void move(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub.publish(msg);
////    }
////    void move_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub_quad.publish(msg);
////    }


////    void turn(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub.publish(msg);
////    }


////    void turn_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub_quad.publish(msg);
////    }



////    void init_move(double val)
////    {
////        for ( int i = 0 ; i < 1000000 ; i++ )
////        {
////            geometry_msgs::Twist msg;
////            msg.linear.x = 0.0;
////            msg.linear.y =  0.0 ;
////            msg.linear.z =  val;
////            msg.angular.x = 0.0 ;
////            msg.angular.y = 0.0 ;
////            msg.angular.z = 0.0 ;
////            vel_pub_quad.publish(msg);

////        }
////        geometry_msgs::Twist msg;
////        msg.linear.x = 0.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = 0.0 ;
////        vel_pub_quad.publish(msg);

////    }

////private:
////    // ROS
////    ros::NodeHandle n;
////    ros::Publisher vel_pub ;
////    ros::Publisher vel_pub_quad ;

////} ;


////int main(int argc, char **argv)
////{

////    // ROS node

////    ros::init(argc, argv, "move_robot");
////    ros::NodeHandle n;
////    ros::NodeHandle n_priv("~");
////    ros::Rate loop_rate(150);

////    // create an object
////    move_robot move_robot_obj_1(n);
////    move_robot move_robot_obj_2(n);

////    move_robot_obj_2.init_move(0.3) ;

////    while(ros::ok())
////    {


////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn left" << std::endl ;
////                move_robot_obj_1.turn(0.780);
////                move_robot_obj_2.turn_2(-0.9);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <9999; i++)
////            {
////                std::cout << "move forward" << std::endl ;
////                 move_robot_obj_1.move(1.0);
////                move_robot_obj_2.move_2(0.8);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <9999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(-1.2);
////                move_robot_obj_2.turn_2(1.2);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99990; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(0.8);

////            }

////            loop_rate.sleep();
////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(0.78);
//////
////                move_robot_obj_2.turn_2(-0.78);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(0.8);

////            }

////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(-0.78);
////                move_robot_obj_2.turn_2(0.78);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(0.8);

////            }



////    }

//////    }
////    return 0;
////}







////////////////////////////################### moving not static
////#include "ros/ros.h"
////#include "std_msgs/String.h"
////#include "nav_msgs/Odometry.h"
////#include "sensor_msgs/PointCloud.h"
////#include <visualization_msgs/Marker.h>
////#include <geometry_msgs/Twist.h>
////#include <geometry_msgs/Pose.h>

////#include <visualization_msgs/MarkerArray.h>
////#include <cmath>
////#include <phantom_omni/OmniFeedback.h>
////#include <dynamic_reconfigure/server.h>
////#include <haptic_teleoperation/TwistArray.h>
////#include <haptic_teleoperation/ContourData.h>
////#include <laser_geometry/laser_geometry.h>
////#include <tf/transform_listener.h>
////#include <haptic_teleoperation/potential_fieldConfig.h>
////#include <phantom_omni/PhantomButtonEvent.h>
////#include <gazebo_msgs/ModelState.h>
////#include <Eigen/Eigen>
////#include <Eigen/Geometry>

//////const double M_PI=3.14159265359 ;
////const double deg_to_rad = M_PI / 180.0 ;
////bool flag = true ;
////class move_robot
////{
////public:
////    move_robot(ros::NodeHandle & n_) : n(n_)
////    {
////        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
////        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

////    };


////    void move(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub.publish(msg);
////    }
////    void move_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub_quad.publish(msg);
////    }


////    void turn(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub.publish(msg);
////    }


////    void turn_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub_quad.publish(msg);
////    }



////    void init_move(double val)
////    {
////        for ( int i = 0 ; i < 1000000 ; i++ )
////        {
////            geometry_msgs::Twist msg;
////            msg.linear.x = 0.0;
////            msg.linear.y =  0.0 ;
////            msg.linear.z =  val;
////            msg.angular.x = 0.0 ;
////            msg.angular.y = 0.0 ;
////            msg.angular.z = 0.0 ;
////            vel_pub_quad.publish(msg);

////        }
////        geometry_msgs::Twist msg;
////        msg.linear.x = 0.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = 0.0 ;
////        vel_pub_quad.publish(msg);

////    }

////private:
////    // ROS
////    ros::NodeHandle n;
////    ros::Publisher vel_pub ;
////    ros::Publisher vel_pub_quad ;

////} ;


////int main(int argc, char **argv)
////{

////    // ROS node

////    ros::init(argc, argv, "move_robot");
////    ros::NodeHandle n;
////    ros::NodeHandle n_priv("~");
////    ros::Rate loop_rate(150);

////    // create an object
////    move_robot move_robot_obj_1(n);
////    move_robot move_robot_obj_2(n);

////    move_robot_obj_2.init_move(2.0) ;

////    while(ros::ok())
////    {


////            for (int i=0 ;i <99999; i++)
////            {
////               // std::cout << "turn left" << std::endl ;
////                move_robot_obj_1.turn(-0.780);
////                move_robot_obj_2.turn_2(0.780);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <9999; i++)
////            {
////                //std::cout << "move forward" << std::endl ;
////                  move_robot_obj_1.move(1.0);
////                move_robot_obj_2.move_2(0.8);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////              //  std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(1.2);
////                move_robot_obj_2.turn_2(-1.2);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////               // std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(1.0);

////            }

////            loop_rate.sleep();
////            for (int i=0 ;i <99999; i++)
////            {
////               // std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(-1.78);
////                move_robot_obj_2.turn_2(0.78);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////              //  std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.1);

////                move_robot_obj_2.move_2(1.2);

////            }

////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////              //  std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.turn(0.78);
////                move_robot_obj_2.turn_2(-0.78);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////            //    std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(1.0);

////            }



////    }

//////    }
////    return 0;
////}









////////////////////////################### moving not static
////#include "ros/ros.h"
////#include "std_msgs/String.h"
////#include "nav_msgs/Odometry.h"
////#include "sensor_msgs/PointCloud.h"
////#include <visualization_msgs/Marker.h>
////#include <geometry_msgs/Twist.h>
////#include <geometry_msgs/Pose.h>

////#include <visualization_msgs/MarkerArray.h>
////#include <cmath>
////#include <phantom_omni/OmniFeedback.h>
////#include <dynamic_reconfigure/server.h>
////#include <haptic_teleoperation/TwistArray.h>
////#include <haptic_teleoperation/ContourData.h>
////#include <laser_geometry/laser_geometry.h>
////#include <tf/transform_listener.h>
////#include <haptic_teleoperation/potential_fieldConfig.h>
////#include <phantom_omni/PhantomButtonEvent.h>
////#include <gazebo_msgs/ModelState.h>
////#include <Eigen/Eigen>
////#include <Eigen/Geometry>

//////const double M_PI=3.14159265359 ;
////const double deg_to_rad = M_PI / 180.0 ;
////bool flag = true ;
////class move_robot
////{
////public:
////    move_robot(ros::NodeHandle & n_) : n(n_)
////    {
////        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
////        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

////    };


////    void move(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub.publish(msg);
////    }
////    void move_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = val;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0 ;
////        msg.angular.y = 0 ;
////        msg.angular.z = 0 ;
////        vel_pub_quad.publish(msg);
////    }


////    void turn(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub.publish(msg);
////    }


////    void turn_2(double val)
////    {
////        geometry_msgs::Twist msg;
////        msg.linear.x = 1.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = val ;
////        vel_pub_quad.publish(msg);
////    }



////    void init_move(double val)
////    {
////        for ( int i = 0 ; i < 1000000 ; i++ )
////        {
////            geometry_msgs::Twist msg;
////            msg.linear.x = 0.0;
////            msg.linear.y =  0.0 ;
////            msg.linear.z =  val;
////            msg.angular.x = 0.0 ;
////            msg.angular.y = 0.0 ;
////            msg.angular.z = 0.0 ;
////            vel_pub_quad.publish(msg);

////        }
////        geometry_msgs::Twist msg;
////        msg.linear.x = 0.0;
////        msg.linear.y =  0.0 ;
////        msg.linear.z =  0.0;
////        msg.angular.x = 0.0 ;
////        msg.angular.y = 0.0 ;
////        msg.angular.z = 0.0 ;
////        vel_pub_quad.publish(msg);

////    }

////private:
////    // ROS
////    ros::NodeHandle n;
////    ros::Publisher vel_pub ;
////    ros::Publisher vel_pub_quad ;

////} ;


////int main(int argc, char **argv)
////{

////    // ROS node

////    ros::init(argc, argv, "move_robot");
////    ros::NodeHandle n;
////    ros::NodeHandle n_priv("~");
////    ros::Rate loop_rate(150);

////    // create an object
////    move_robot move_robot_obj_1(n);
////    move_robot move_robot_obj_2(n);

////    move_robot_obj_2.init_move(2.0) ;

////    while(ros::ok())
////    {


////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn left" << std::endl ;
////                move_robot_obj_1.move(0.780);
////                move_robot_obj_2.turn_2(-0.780);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move forward" << std::endl ;
////                  move_robot_obj_1.move(1.0);
////                move_robot_obj_2.move_2(0.8);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.move(0.2);
////                move_robot_obj_2.turn_2(1.2);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(1.0);

////            }

////            loop_rate.sleep();
////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.move(0.78);

////                move_robot_obj_2.turn_2(-0.78);

////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.1);

////                move_robot_obj_2.move_2(1.2);

////            }

////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "turn right" << std::endl ;
////                move_robot_obj_1.move(0.78);
////                move_robot_obj_2.turn_2(0.78);


////            }
////            loop_rate.sleep();

////            for (int i=0 ;i <99999; i++)
////            {
////                std::cout << "move backward" << std::endl ;
////                move_robot_obj_1.move(1.0);

////                move_robot_obj_2.move_2(1.0);

////            }



////    }

//////    }
////    return 0;
////}

////#include "ros/ros.h"
////#include "std_msgs/String.h"
////#include "nav_msgs/Odometry.h"
////#include "sensor_msgs/PointCloud.h"
////#include <visualization_msgs/Marker.h>
////#include <geometry_msgs/Twist.h>
////#include <geometry_msgs/Pose.h>

////#include <visualization_msgs/MarkerArray.h>
////#include <cmath>
////#include <phantom_omni/OmniFeedback.h>
////#include <dynamic_reconfigure/server.h>
////#include <haptic_teleoperation/TwistArray.h>
////#include <haptic_teleoperation/ContourData.h>
////#include <laser_geometry/laser_geometry.h>
////#include <tf/transform_listener.h>
////#include <haptic_teleoperation/potential_fieldConfig.h>
////#include <phantom_omni/PhantomButtonEvent.h>
////#include <gazebo_msgs/ModelState.h>
////#include <Eigen/Eigen>
////#include <Eigen/Geometry>

//////const double M_PI=3.14159265359 ;
////const double deg_to_rad = M_PI / 180.0 ;
////bool flag = true ;
////class move_robot
////{
////public:
////   move_robot(ros::NodeHandle & n_) : n(n_)
////   {
////       vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
////       vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

////   };


////   void move(double val)
////   {
////       geometry_msgs::Twist msg;
////       msg.linear.x = val;
////       msg.linear.y =  0.0 ;
////       msg.linear.z =  0.0;
////       msg.angular.x = 0 ;
////       msg.angular.y = 0 ;
////       msg.angular.z = 0 ;
////       vel_pub.publish(msg);
////   }
////   void move_2(double val)
////   {
////       geometry_msgs::Twist msg;
////       msg.linear.x = val;
////       msg.linear.y =  0.0 ;
////       msg.linear.z =  0.0;
////       msg.angular.x = 0 ;
////       msg.angular.y = 0 ;
////       msg.angular.z = 0 ;
////       vel_pub_quad.publish(msg);
////   }


////   void turn(double val)
////   {
////       geometry_msgs::Twist msg;
////       msg.linear.x = 1.0;
////       msg.linear.y =  0.0 ;
////       msg.linear.z =  0.0;
////       msg.angular.x = 0.0 ;
////       msg.angular.y = 0.0 ;
////       msg.angular.z = val ;
////       vel_pub.publish(msg);
////   }


////   void turn_2(double val)
////   {
////       geometry_msgs::Twist msg;
////       msg.linear.x = 1.0;
////       msg.linear.y =  0.0 ;
////       msg.linear.z =  0.0;
////       msg.angular.x = 0.0 ;
////       msg.angular.y = 0.0 ;
////       msg.angular.z = val ;
////       vel_pub_quad.publish(msg);
////   }



////   void init_move(double val)
////   {
////       for ( int i = 0 ; i < 1000000 ; i++ )
////       {
////           geometry_msgs::Twist msg;
////           msg.linear.x = 0.0;
////           msg.linear.y =  0.0 ;
////           msg.linear.z =  val;
////           msg.angular.x = 0.0 ;
////           msg.angular.y = 0.0 ;
////           msg.angular.z = 0.0 ;
////           vel_pub_quad.publish(msg);

////       }
////       geometry_msgs::Twist msg;
////       msg.linear.x = 0.0;
////       msg.linear.y =  0.0 ;
////       msg.linear.z =  0.0;
////       msg.angular.x = 0.0 ;
////       msg.angular.y = 0.0 ;
////       msg.angular.z = 0.0 ;
////       vel_pub_quad.publish(msg);

////   }

////private:
////   // ROS
////   ros::NodeHandle n;
////   ros::Publisher vel_pub ;
////   ros::Publisher vel_pub_quad ;

////} ;


////int main(int argc, char **argv)
////{

////   // ROS node
////   ros::init(argc, argv, "move_robot");
////   ros::NodeHandle n;
////   ros::NodeHandle n_priv("~");
////   ros::Rate loop_rate(150);

////   // create an object
////   move_robot move_robot_obj_1(n);
////   move_robot move_robot_obj_2(n);

////   move_robot_obj_2.init_move(2.0) ;

////   while(ros::ok())
////   {


////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "turn left" << std::endl ;
////               move_robot_obj_1.turn(0.780);
////               move_robot_obj_2.turn_2(-0.780);

////           }
////           loop_rate.sleep();

////           for (int i=0 ;i <9999; i++)
////           {
////               std::cout << "move forward" << std::endl ;
////                 move_robot_obj_1.move(1.0);
////                 move_robot_obj_2.move_2(0.8);


////           }
////           loop_rate.sleep();

////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "turn right" << std::endl ;
////               move_robot_obj_1.turn(-1.2);
////              move_robot_obj_2.turn_2(1.2);


////           }
////           loop_rate.sleep();

////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "move backward" << std::endl ;
////               move_robot_obj_1.move(0.8);

////               move_robot_obj_2.move_2(1.0);

////           }

////           loop_rate.sleep();
////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "turn right" << std::endl ;
////               move_robot_obj_1.turn(1.2);

////               move_robot_obj_2.turn_2(-1.2);

////           }
////           loop_rate.sleep();

////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "move backward" << std::endl ;
////               move_robot_obj_1.move(1.0);

////              move_robot_obj_2.move_2(1.0);

////           }

////           loop_rate.sleep();

////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "turn right" << std::endl ;
////               move_robot_obj_1.turn(-1.2);
////               move_robot_obj_2.turn_2(0.9);


////           }
////           loop_rate.sleep();

////           for (int i=0 ;i <99999; i++)
////           {
////               std::cout << "move backward" << std::endl ;
////               move_robot_obj_1.move(1.0);

////           //    move_robot_obj_2.move_2(1.0);

////           }

////           loop_rate.sleep();


////   }

//////    }
////   return 0;
////}






//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "nav_msgs/Odometry.h"
//#include "sensor_msgs/PointCloud.h"
//#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose.h>

//#include <visualization_msgs/MarkerArray.h>
//#include <cmath>
//#include <phantom_omni/OmniFeedback.h>
//#include <dynamic_reconfigure/server.h>
//#include <haptic_teleoperation/TwistArray.h>
//#include <haptic_teleoperation/ContourData.h>
//#include <laser_geometry/laser_geometry.h>
//#include <tf/transform_listener.h>
//#include <haptic_teleoperation/potential_fieldConfig.h>
//#include <phantom_omni/PhantomButtonEvent.h>
//#include <gazebo_msgs/ModelState.h>
//#include <Eigen/Eigen>
//#include <Eigen/Geometry>

////const double M_PI=3.14159265359 ;
//const double deg_to_rad = M_PI / 180.0 ;
//bool flag = true ;
//class move_robot
//{
//public:
//    move_robot(ros::NodeHandle & n_) : n(n_)
//    {
//        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
//        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

//    }


//    void move(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = val;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0 ;
//        msg.angular.y = 0 ;
//        msg.angular.z = 0 ;
//        vel_pub.publish(msg);
//    }
//    void move_2(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = val;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0 ;
//        msg.angular.y = 0 ;
//        msg.angular.z = 0 ;
//        vel_pub_quad.publish(msg);
//    }


//    void turn(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = 0.3;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = val ;
//        vel_pub.publish(msg);
//    }


//    void turn_2(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = 0.3;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = val ;
//        vel_pub_quad.publish(msg);
//    }



//    void init_move(double val)
//    {
//        for ( int i = 0 ; i < 1000000 ; i++ )
//        {
//            geometry_msgs::Twist msg;
//            msg.linear.x = 0.0;
//            msg.linear.y =  0.0 ;
//            msg.linear.z =  val;
//            msg.angular.x = 0.0 ;
//            msg.angular.y = 0.0 ;
//            msg.angular.z = 0.0 ;
//            vel_pub_quad.publish(msg);

//        }
//        geometry_msgs::Twist msg;
//        msg.linear.x = 0.0;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = 0.0 ;
//        vel_pub_quad.publish(msg);

//    }

//private:
//    // ROS
//    ros::NodeHandle n;
//    ros::Publisher vel_pub ;
//    ros::Publisher vel_pub_quad ;

//} ;


//int main(int argc, char **argv)
//{

//    // ROS node

//    ros::init(argc, argv, "move_robot");
//    ros::NodeHandle n;
//    ros::NodeHandle n_priv("~");
//    ros::Rate loop_rate(150);

//    // create an object
//    move_robot move_robot_obj_1(n);
//    move_robot move_robot_obj_2(n);

//    move_robot_obj_2.init_move(2.0) ;

//    while(ros::ok())
//    {


//            for (long int i=0 ;i<99999; i++)
//            {
//                std::cout << "turn left" << std::endl ;
//              //  move_robot_obj_1.turn(0.787);
//                move_robot_obj_2.turn_2(-0.787);
//            }
//            loop_rate.sleep();



//            for (int i=0 ;i <400009; i++)
//            {
//                std::cout << "move forward" << std::endl ;
//             //   move_robot_obj_1.move(1.0);
//                move_robot_obj_2.move_2(1.2);
//            }
//           loop_rate.sleep();


//            for (int i=0 ;i <99999; i++)
//            {
//                std::cout << "turn right" << std::endl ;
//              //  move_robot_obj_1.turn(-1.5);
//                move_robot_obj_2.turn_2(1.3);


//            }
//           loop_rate.sleep();

//            for (int i=0 ;i <99999; i++)
//            {
//                std::cout << "move backward" << std::endl ;
//              //  move_robot_obj_1.move(1.0);

//                move_robot_obj_2.move_2(1.0);

//            }

//            loop_rate.sleep();
//            for (int i=0 ;i <99999; i++)
//            {
//                std::cout << "turn right" << std::endl ;
//             //   move_robot_obj_1.turn(1.5);

//                move_robot_obj_2.turn_2(-1.5);

//            }
//            loop_rate.sleep();

//            for (int i=0 ;i <999999; i++)
//            {
//                std::cout << "move backward" << std::endl ;
//              //  move_robot_obj_1.move(1.0);

//                move_robot_obj_2.move_2(1.0);

//            }

//            loop_rate.sleep();

//            for (int i=0 ;i <99999; i++)
//            {
//                std::cout << "turn right" << std::endl ;
//              //  move_robot_obj_1.turn(-1.3);
//                move_robot_obj_2.turn_2(1.3);


//            }
//           loop_rate.sleep();

//            for (int i=0 ;i <99999; i++)
//            {
//                std::cout << "move backward" << std::endl ;
//              //  move_robot_obj_1.move(1.0);

//                move_robot_obj_2.move_2(1.0);

//            }

//            loop_rate.sleep();


//    }

////    }
//    return 0;
//}


////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "turn left" << std::endl ;
////  //  move_robot_obj_1.turn(0.787);
////    move_robot_obj_2.turn_2(-0.787);
////}
////loop_rate.sleep();



////for (int i=0 ;i <400009; i++)
////{
////    std::cout << "move forward" << std::endl ;
//// //   move_robot_obj_1.move(1.0);
////    move_robot_obj_2.move_2(1.2);
////}
////loop_rate.sleep();


////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "turn right" << std::endl ;
////  //  move_robot_obj_1.turn(-1.5);
////    move_robot_obj_2.turn_2(1.3);


////}
////loop_rate.sleep();

////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "move backward" << std::endl ;
////  //  move_robot_obj_1.move(1.0);

////    move_robot_obj_2.move_2(1.0);

////}

////loop_rate.sleep();
////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "turn right" << std::endl ;
//// //   move_robot_obj_1.turn(1.5);

////    move_robot_obj_2.turn_2(-1.5);

////}
////loop_rate.sleep();

////for (int i=0 ;i <999999; i++)
////{
////    std::cout << "move backward" << std::endl ;
////  //  move_robot_obj_1.move(1.0);

////    move_robot_obj_2.move_2(1.0);

////}

////loop_rate.sleep();

////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "turn right" << std::endl ;
////  //  move_robot_obj_1.turn(-1.3);
////    move_robot_obj_2.turn_2(1.3);


////}
////loop_rate.sleep();

////for (int i=0 ;i <99999; i++)
////{
////    std::cout << "move backward" << std::endl ;
////  //  move_robot_obj_1.move(1.0);

////    move_robot_obj_2.move_2(1.0);

////}



////}

//////    }
////return 0;




// This the one used for regenerating the data after changing the code //
//

//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "nav_msgs/Odometry.h"
//#include "sensor_msgs/PointCloud.h"
//#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <cmath>
//#include <phantom_omni/OmniFeedback.h>
//#include <dynamic_reconfigure/server.h>
//#include <haptic_teleoperation/TwistArray.h>
//#include <haptic_teleoperation/ContourData.h>
//#include <laser_geometry/laser_geometry.h>
//#include <tf/transform_listener.h>
//#include <haptic_teleoperation/potential_fieldConfig.h>
//#include <phantom_omni/PhantomButtonEvent.h>
//#include <gazebo_msgs/ModelState.h>
//#include <Eigen/Eigen>
//#include <Eigen/Geometry>

//#define const double M_PI=3.141592653 ;
////#define const double deg_to_rad = M_PI / 180.0 ;
//bool flag = true ;
//class move_robot
//{
//public:
//    move_robot(ros::NodeHandle & n_) : n(n_)
//    {
//        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
//        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

//    }


//    void move(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = val;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0 ;
//        msg.angular.y = 0 ;
//        msg.angular.z = 0 ;
//        vel_pub.publish(msg);
//    }
//    void move_2(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = val;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0 ;
//        msg.angular.y = 0 ;
//        msg.angular.z = 0 ;
//        vel_pub_quad.publish(msg);
//    }


//    void turn(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = 1.0;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = val ;
//        vel_pub.publish(msg);
//    }


//    void turn_2(double val)
//    {
//        geometry_msgs::Twist msg;
//        msg.linear.x = 1.0;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = val ;
//        vel_pub_quad.publish(msg);
//    }



//    void init_move(double val)
//    {
//        for ( int i = 0 ; i < 1000000 ; i++ )
//        {
//            geometry_msgs::Twist msg;
//            msg.linear.x = 0.0;
//            msg.linear.y =  0.0 ;
//            msg.linear.z =  val;
//            msg.angular.x = 0.0 ;
//            msg.angular.y = 0.0 ;
//            msg.angular.z = 0.0 ;
//            vel_pub_quad.publish(msg);

//        }
//        geometry_msgs::Twist msg;
//        msg.linear.x = 0.0;
//        msg.linear.y =  0.0 ;
//        msg.linear.z =  0.0;
//        msg.angular.x = 0.0 ;
//        msg.angular.y = 0.0 ;
//        msg.angular.z = 0.0 ;
//        vel_pub_quad.publish(msg);

//    }

//private:
//    // ROS
//    ros::NodeHandle n;
//    ros::Publisher vel_pub ;
//    ros::Publisher vel_pub_quad ;

//} ;


//int main(int argc, char **argv)
//{

//    // ROS node

//    ros::init(argc, argv, "move_robot");
//    ros::NodeHandle n;
//    ros::NodeHandle n_priv("~");
//    ros::Rate loop_rate(1);

//    // create an object
//    move_robot move_robot_obj_2(n);
//    move_robot move_robot_obj_1(n);
//    move_robot_obj_2.init_move(2.0);
//    ros::Duration(0.5).sleep();
//    move_robot_obj_2.init_move(0.0);
//    ros::Duration(0.5).sleep();

//    while(ros::ok())
//    {


//        ros::Time start_time1 = ros::Time::now();
//        ros::Duration timeout1(0.5); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time1 < timeout1) {
//            move_robot_obj_1.turn(0.787);
//            move_robot_obj_2.turn_2(-0.787);


//        }


//        ros::Time start_time2 = ros::Time::now();
//        ros::Duration timeout2(7.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time2 < timeout2) {
//            move_robot_obj_1.move(0.9);
//            move_robot_obj_2.move_2(0.8);


//        }



//        ros::Time start_time3 = ros::Time::now();
//        ros::Duration timeout3(1.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time3 < timeout3) {
//            move_robot_obj_1.turn(-1.4);
//            move_robot_obj_2.turn_2(1.4);
//        }



//        ros::Time start_time5 = ros::Time::now();
//        ros::Duration timeout5(3.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time5 < timeout5) {
//            move_robot_obj_1.move(0.8);
//            move_robot_obj_2.move_2(1.1);


//        }

//        ros::Time start_time4 = ros::Time::now();
//        ros::Duration timeout4(1.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time4 < timeout4) {
//            move_robot_obj_1.turn(2.5);
//            move_robot_obj_2.turn_2(-1.6);
//        }


//        ros::Time start_time6 = ros::Time::now();
//        ros::Duration timeout6(4.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time6 < timeout6) {
//            move_robot_obj_1.move(1.0);
//            move_robot_obj_2.move_2(0.7);


//        }


//        ros::Time start_time7 = ros::Time::now();
//        ros::Duration timeout7(1.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time7 < timeout7) {
//            move_robot_obj_1.turn(-3.0);
//            move_robot_obj_2.turn_2(1.6);
//        }


//        ros::Time start_time8 = ros::Time::now();
//        ros::Duration timeout8(5.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time8 < timeout8) {
//            move_robot_obj_1.move(1.0);
//            move_robot_obj_2.move_2(1.0);


//        }



//        ros::Time start_time9 = ros::Time::now();
//        ros::Duration timeout9(5.0); // Timeout of 2 seconds
//        while(ros::Time::now() - start_time9 < timeout9) {
//            move_robot_obj_1.move(0.0);
//            move_robot_obj_2.move_2(0.0);


//        }

//        //        std::cout << "turn right" << std::endl ;
//        //      //  move_robot_obj_1.turn(1.2);
//        //        move_robot_obj_2.turn_2(-0.78);
//        //        ros::Duration(1.0).sleep();

//        //      //  move_robot_obj_1.move(1.1);
//        //        move_robot_obj_2.move_2(1.2);
//        //        ros::Duration(3.0).sleep();


//        //        std::cout << "turn right" << std::endl ;
//        //       // move_robot_obj_1.turn(-0.78);
//        //        move_robot_obj_2.turn_2(0.78);
//        //        ros::Duration(1.0).sleep();


//        //        std::cout << "move backward" << std::endl ;
//        //      //  move_robot_obj_1.move(1.0);
//        //  move_robot_obj_2.move_2(1.0);
//        //        ros::Duration(3.0).sleep();


//    }

//    //    }
//    return 0;
//}



// I am trying to move the robot without turning the UAV

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

#define const double M_PI=3.141592653 ;
//#define const double deg_to_rad = M_PI / 180.0 ;
bool flag = true ;
class move_robot
{
public:
    move_robot(ros::NodeHandle & n_) : n(n_)
    {
        vel_pub = n.advertise<geometry_msgs::Twist>("/husky_ns/husky/cmd_vel", 1);
        vel_pub_quad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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

    void move_2(double val_x ,double val_y)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = val_x;
        msg.linear.y = val_y ;
        msg.linear.z =  0.0;
        msg.angular.x = 0 ;
        msg.angular.y = 0 ;
        msg.angular.z = 0 ;
        vel_pub_quad.publish(msg);
    }



    void turn(double val)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
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
            vel_pub_quad.publish(msg);

        }
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y =  0.0 ;
        msg.linear.z =  0.0;
        msg.angular.x = 0.0 ;
        msg.angular.y = 0.0 ;
        msg.angular.z = 0.0 ;
        vel_pub_quad.publish(msg);

    }

private:
    // ROS
    ros::NodeHandle n;
    ros::Publisher vel_pub ;
    ros::Publisher vel_pub_quad ;

} ;


int main(int argc, char **argv)
{

    // ROS node

    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(1);

    // create an object
    move_robot move_robot_obj_2(n);
    move_robot move_robot_obj_1(n);
    move_robot_obj_2.init_move(2.0);
    ros::Duration(0.5).sleep();
    move_robot_obj_2.init_move(0.0);
    ros::Duration(0.5).sleep();

    while(ros::ok())
    {


        ros::Time start_time1 = ros::Time::now();
        ros::Duration timeout1(0.5); // Timeout of 2 seconds
        while(ros::Time::now() - start_time1 < timeout1) {
            move_robot_obj_1.turn(0.787);

        }


        ros::Time start_time2 = ros::Time::now();
        ros::Duration timeout2(7.5); // Timeout of 2 seconds
        while(ros::Time::now() - start_time2 < timeout2) {
            move_robot_obj_1.move(0.5);
            move_robot_obj_2.move_2(0.5 , -0.5);

        }



        ros::Time start_time9 = ros::Time::now();
        ros::Duration timeout9(5.0); // Timeout of 2 seconds
        while(ros::Time::now() - start_time9 < timeout9) {
            move_robot_obj_1.move(0.0);
            move_robot_obj_2.move_2(0.0,0.0);
        }


        ros::Time start_time3 = ros::Time::now();
        ros::Duration timeout3(1.5); // Timeout of 2 seconds
        while(ros::Time::now() - start_time3 < timeout3) {
            //move_robot_obj_1.turn(-1.574);
            move_robot_obj_1.turn(-1.9);

            move_robot_obj_2.move_2(0 , 0);

        }



        ros::Time start_time5 = ros::Time::now();
        ros::Duration timeout5(7.0); // Timeout of 2 seconds
        while(ros::Time::now() - start_time5 < timeout5) {
            move_robot_obj_1.move(0.5);
            move_robot_obj_2.move_2(0.5 , 0.5);


        }

        ros::Time start_time99 = ros::Time::now();
        ros::Duration timeout99(5.0); // Timeout of 2 seconds
        while(ros::Time::now() - start_time99 < timeout99) {
            move_robot_obj_1.move(0.0);
            move_robot_obj_2.move_2(0.0,0.0);
        }

        exit(1) ;

    }

    return 0;
}



