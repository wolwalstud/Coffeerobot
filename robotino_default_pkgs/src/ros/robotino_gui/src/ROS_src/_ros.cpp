#include "_ros.h"
#include "std_msgs/UInt16.h"


_Ros::_Ros()
{
    n = new ros::NodeHandle();
    turtle_move = n->advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    pos_name = n->advertise<std_msgs::String>("pos_name", 1);
    csv_close = n->advertise<std_msgs::Bool>("/csv_closed",1000);
    csv_goal=n->advertise<std_msgs::String>("/csv_goal",1000);
    pose_sub = n->subscribe("amcl_pose",100,amcl_pose_callback);
    vel_sub = n->subscribe("odom",100,odom_callback);

    

    //linear_x = 1;
}
_Ros::~_Ros()
{
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    delete n;
}
void _Ros::twist_publisher()
{   
        
    turtle_move.publish(twist_msg);
 
}

void _Ros::posname_publisher(std_msgs::String pos_input)
{
    pos_name.publish(pos_input);
}

void _Ros::closecsv_publisher(std_msgs::Bool closed)
{
    csv_close.publish(closed);
}


void _Ros::csvgoal_publisher(std_msgs::String goal_input)
{
    csv_goal.publish(goal_input);
}


float _Ros::makeSimpleProfile(float output, float input, float slop){
    if (input > output){
        output = std::min(input, output + slop);
    }        
    else if (input < output){
        output = std::max(input, output - slop );
    }
    else{
        output = input;
    }

    return output;
}



