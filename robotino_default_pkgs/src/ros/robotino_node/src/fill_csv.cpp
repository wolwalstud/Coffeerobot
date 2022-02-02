
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>







class CSV{

public: 


	void createCSV(std::ofstream& myfile);


	void fillCSV(std::ofstream& myfile);


	void readJointStateMessages(const sensor_msgs::JointStateConstPtr& msg_joint);


	void readAMCLpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);

	void posnameCallback(const std_msgs::String::ConstPtr& msg);

	void closecsvCallback(const std_msgs::Bool::ConstPtr& kill);

	bool closeCSV();

private: 

	float pos_x = 0.0;          /**< Storage for robots position on the x axis */
	float pos_y = 0.0;			/**< Storage for robots position on the y axis */
	float orientation_x = 0.0;
	float orientation_y = 0.0;
	float orientation_z = 0.0;
	float orientation_w = 0.0;			/**< Storage for robots velocity in the x direction*/
	float vel_y = 0.0;			/**< Storage for robots velocity in the y direction*/
	float vel_left_we = 0.0;	/**< Storage for robots velocity of the left wheel*/
	float vel_right_we = 0.0;	//**< Storage for robots velocity of the right wheel*/
	float header_time = 0.0;	/**< Storage for the time of the simulation*/
	double roll, pitch, yaw;	/**< Storage for roll pitch and yaw of the robot*/
	std::string oldname;
	bool close_csv=false;
	std::string pos_name;

	//String name;

};


void CSV::posnameCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  pos_name=msg->data.c_str();

}

void CSV::closecsvCallback(const std_msgs::Bool::ConstPtr& kill)
{
  close_csv=kill->data;
  
}


void CSV::readJointStateMessages(const sensor_msgs::JointStateConstPtr& msg_joint)
{
	vel_left_we=msg_joint->velocity[0];
	vel_right_we=msg_joint->velocity[1];
}


void CSV::readAMCLpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
	pos_x = msgAMCL->pose.pose.position.x;
	pos_y = msgAMCL->pose.pose.position.y;

	//vel_x = msgAMCL->twist.twist.linear.x;
	//vel_y = msgAMCL->twist.twist.linear.y;

	//tf::Quaternion q(
    orientation_x = msgAMCL->pose.pose.orientation.x;
    orientation_y = msgAMCL->pose.pose.orientation.y;
	orientation_z = msgAMCL->pose.pose.orientation.z;
	orientation_w = msgAMCL->pose.pose.orientation.w;
    //tf::Matrix3x3 m(q);
    
    //m.getRPY(roll, pitch, yaw);
    
    header_time =msgAMCL->header.stamp.sec;
}


bool CSV::closeCSV(){
	
	if(close_csv==true){
		return true;
	}else{
		return false;
	}

}


void CSV::createCSV(std::ofstream& myfile){

	std::string position_csv = CSV_PATH;
	myfile.open(position_csv);
	
}


void CSV::fillCSV(std::ofstream& myfile){
	
	
	if(pos_name.empty()==false && pos_name!=oldname){
		myfile << pos_name << "," << std::to_string(pos_x)<<","<<std::to_string(pos_y)<<","<<std::to_string(orientation_x)<<","<<std::to_string(orientation_y)<<","<<std::to_string(orientation_z)<<","<<std::to_string(orientation_w)<<","<<std::to_string(header_time)<<"\n";
		std::cout<< pos_name <<"," <<std::to_string(pos_x)<<","<<std::to_string(pos_y)<<","<<std::to_string(yaw)<<","<<std::to_string(header_time)<<"\n";
		
	}
	oldname=pos_name;
}



int main(int argc, char** argv)
{	
	CSV file;
	ros::init(argc, argv, "fill_csv");
	ros::NodeHandle nh("~");
	std::cout<<"Bin am Ende"<< std::endl;
	ros::Subscriber sub = nh.subscribe("/amcl_pose", 1000, &CSV::readAMCLpose, &file);
	ros::Subscriber kill_sub=nh.subscribe("/csv_closed", 1000, &CSV::closecsvCallback, &file);
	ros::Subscriber posname_sub=nh.subscribe("/pos_name", 1000, &CSV::posnameCallback, &file);
	//ros::Subscriber sub_joints = nh.subscribe("/joint_states", 1000, &CSV::readJointStateMessages, &file);
	ros::Rate loop_rate(5);
	std::ofstream myfile;
	
	
	file.createCSV(myfile);

	int count=1;
	while(ros::ok())
	{	
		

	
		
		file.fillCSV(myfile);
		
		if (file.closeCSV()){
			std::cout<<"Bin am Ende"<< std::endl;
			myfile.close();
			std::cout<<"Bin am Ende2"<< std::endl;
			ros::shutdown();
			return 0;
		}
		

		ros::spinOnce();
		loop_rate.sleep();
	}

	myfile.close();
    return 0;
}


 



 
