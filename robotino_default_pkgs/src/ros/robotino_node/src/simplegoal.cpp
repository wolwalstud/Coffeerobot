
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <stdlib.h>
//http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalHandler{

	public: 
		
		GoalHandler() : ac("move_base",true){}; //tell the action client that we want to spin a thread by default
		void ACServerActive(float time);

		void createNextGoal(float position[], float orientation[]);

		void ExecuteGoal();

	private:

		MoveBaseClient ac; //action client
		move_base_msgs::MoveBaseGoal goal; //goal to publish

};

/*!
     *  This function waits for the action server to come up
     *  \param time	Input of the duration time
*/
void GoalHandler::ACServerActive(float time){
	while(!ac.waitForServer(ros::Duration(time))){ //wait for the action server to come up
    			ROS_INFO("Waiting for the move_base action server to come up");
  			}
}

/*!
     *  This function sends the next goal and tests if the robot reached the goal.
*/
void GoalHandler::ExecuteGoal(){
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
}

/*!
     *  This function creates the new goal which should be published next.
     *  \param position      Input of next goal position
     *  \param orientation   Input of next goal orientation     */
void GoalHandler::createNextGoal(float position[], float orientation[])
{
	//move_base_msgs::MoveBaseGoal goal; //goal to publish
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now(); 
	
	goal.target_pose.pose.position.x = position[0]; 
	goal.target_pose.pose.position.y = position[1];
	goal.target_pose.pose.position.z = position[2];
	goal.target_pose.pose.orientation.x = orientation[0];
	goal.target_pose.pose.orientation.y = orientation[1];
	goal.target_pose.pose.orientation.z = orientation[2];
	goal.target_pose.pose.orientation.w = orientation[3];  
}

int main(int argc, char** argv){

  ros::init(argc, argv, "simplegoal"); // Init ROS node
  ros::NodeHandle nh("~");
  GoalHandler goalhandler;
  goalhandler.ACServerActive(5.0); //wait for the action server to come up
  
  std::cout << "Argument= " << argv[1];


  if (std::string(argv[1])=="f3.05")
	{
		float position[]={-3.803, -1.175, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, 0.77, 0.638};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
		std::cout<<"Ich komme zum Raum F3.05" << std::endl;
	}
	else if (std::string(argv[1])== "f3.06")
		{
			float position[]={4.0, 1.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
			float orientation[]={ 0.0, 0.0, 0.77, 0.638};
			goalhandler.createNextGoal(position,orientation); //create goal to publish
			std::cout<<"Ich komme zum Raum F3.06" << std::endl;
		}

	
	
    goalhandler.ExecuteGoal(); 
  
    float position[]={0.0, 0.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
	float orientation[]={ 0.0, 0.0, 1.0, 0.0};
	goalhandler.createNextGoal(position,orientation); //create goal to publish
    
	goalhandler.ExecuteGoal();
	std::cout <<"ups das ging zu schnell oder vielleicht doch nicht?" << std::endl;
  //system("$(rospack find soa_mmr3)/launch/kill.sh"); //if every position reached, kill all
  return 0;
}
