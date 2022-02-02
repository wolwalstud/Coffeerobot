
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <ros/package.h>
#include <string>
//http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalHandler{

	public: 
		
		GoalHandler() : ac("move_base",true){}; //tell the action client that we want to spin a thread by default
		void ACServerActive(float time);

		void createNextGoal(/*float position[], float orientation[]*/);

		void ExecuteGoal();

        void read_csv();

        void csvgoalCallback(const std_msgs::String::ConstPtr& msg);

	private:

		MoveBaseClient ac; //action client
		move_base_msgs::MoveBaseGoal goal; //goal to publish

        float posx;
        float posy;
        float orientation_x;
        float orientation_y;
        float orientation_z;
        float orientation_w;
        bool received_goal;
        std::string goal_input;
       

};


void GoalHandler::csvgoalCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  goal_input=msg->data.c_str();

}


/*!
     *  This function waits for the action server to come up
     *  \param time	Input of the duration time
*/
void GoalHandler::ACServerActive(float time){
	while(!ac.waitForServer(ros::Duration(time))&&ros::ok()){ //wait for the action server to come up
    			ROS_INFO("Waiting for the move_base action server to come up");
  			}
}

/*!
     *  This function sends the next goal and tests if the robot reached the goal.
*/
void GoalHandler::ExecuteGoal(){
    if(received_goal==true){
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); //publish goal for robot 
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //print robot succeeded info if goal reached
            ROS_INFO("Hooray, the base moved forward");
        else
        {
            ROS_INFO("The base failed to move forward for some reason"); //print robot failed info if goal reached
            exit(0);
        }
        goal_input.clear();
    }	
}



void GoalHandler::createNextGoal(/*float position[], float orientation[]*/)
{
	//move_base_msgs::MoveBaseGoal goal; //goal to publish
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now(); 
	
	goal.target_pose.pose.position.x = posx; 
	goal.target_pose.pose.position.y = posy;
	goal.target_pose.pose.position.z = 0;
	goal.target_pose.pose.orientation.x = orientation_x;
	goal.target_pose.pose.orientation.y = orientation_y;
	goal.target_pose.pose.orientation.z = orientation_z;
	goal.target_pose.pose.orientation.w = orientation_w;  
}



void GoalHandler::read_csv(){

   
    std::fstream fin;
    int count=0;


    std::string resource_csv = POSITION_PATH;

    //fin.open("/home/wolfram/CATKINWS/src/robotino_default_pkgs/src/ros/robotino_node/position/position.csv", std::ios::in);
    fin.open(resource_csv, std::ios::in);
    std::vector<std::string> row;
    std::string line, word, temp, oldposition;
    received_goal=false;
    if(goal_input.empty()==false){
        received_goal=true; 
        while (std::getline(fin, line)&&ros::ok()) {


            
            row.clear();

            std::stringstream s(line);
        
            
            while (std::getline(s, word, ',')&&ros::ok()) {
                
                // add all the column data
                // of a row to a vector
                row.push_back(word);
                
            }

            
            oldposition = (row[0]);
            
            if (oldposition==goal_input){
                count = 1;
                //move_base_msgs::MoveBaseGoal goal; //goal to publish
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now(); 
                
                goal.target_pose.pose.position.x = std::stof(row[1]); 
                goal.target_pose.pose.position.y = std::stof(row[2]);
                goal.target_pose.pose.position.z = 0;
                goal.target_pose.pose.orientation.x = std::stof(row[3]);
                goal.target_pose.pose.orientation.y = std::stof(row[4]);
                goal.target_pose.pose.orientation.z = std::stof(row[5]);
                goal.target_pose.pose.orientation.w = std::stof(row[6]);
            }
        }
        if (count==0){
            std::cout << "Position nicht gefunden" << std::endl;
        }

    }

}

















int main(int argc, char** argv){

    ros::init(argc, argv, "simplegoal"); // Init ROS node
    ros::NodeHandle nh("~");
    GoalHandler goalhandler;
    ros::Subscriber csvgoal_sub=nh.subscribe("/csv_goal", 1000, &GoalHandler::csvgoalCallback, &goalhandler);
    ros::Rate loop_rate(5);

    //while(true){
    while(ros::ok()){
        std::string target;

        goalhandler.ACServerActive(5.0); //wait for the action server to come up
        goalhandler.read_csv(); 
        goalhandler.ExecuteGoal();
        ros::spinOnce(); 
        loop_rate.sleep();
    }


		

	
  
    
  //system("$(rospack find soa_mmr3)/launch/kill.sh"); //if every position reached, kill all
  return 0;
}
