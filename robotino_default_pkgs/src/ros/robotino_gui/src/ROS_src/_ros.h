#ifndef _ROS_H
#define _ROS_H
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "QDebug"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sstream>
#include "nav_msgs/Odometry.h"


static geometry_msgs::PoseWithCovarianceStamped amcl_pose;
static nav_msgs::Odometry odom_vel;

class _Ros
{
public:
    _Ros();
    ~_Ros();
    //pose topic
    static void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
        amcl_pose = *pose_msg;
    }

    static void odom_callback(const nav_msgs::Odometry::ConstPtr& vel_msg){
        odom_vel = *vel_msg;
    }

    void ultrasonic_subscriber()
    {
        ros::Rate loop_rate(5);
        pose_sub = n->subscribe("amcl_pose",100,amcl_pose_callback);
        vel_sub = n->subscribe("odom",100,odom_callback);
        loop_rate.sleep();
        ros::spinOnce();

    }
    //cmd_vel topic
    void twist_publisher();
    void set_twist(geometry_msgs::Twist twist_input){twist_msg = twist_input;}
    geometry_msgs::Twist get_twist(){return twist_msg;}


    void set_linear_x(float linear_x_input){linear_x = linear_x_input;}
    double get_linear_x(){return linear_x;}

    void posname_publisher(std_msgs::String pos_input);
    void closecsv_publisher(std_msgs::Bool closed);
    void csvgoal_publisher(std_msgs::String goal_input);
    float makeSimpleProfile(float output, float input, float slop);
    

private:


    ros::NodeHandle *n;
    //cmd_vel topic
    ros::Publisher turtle_move;
    geometry_msgs::Twist twist_msg;
    double linear_x;
    //pose topic
    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    //posname topic
    ros::Publisher pos_name;
    ros::Publisher csv_close;
    ros::Publisher csv_goal;

    //std_msgs::String pos_msg;
};

#endif // _ROS_H
