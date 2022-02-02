
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

  ros::init(argc, argv, "simple_navigation_goals"); // Init ROS node
  GoalHandler goalhandler;
  goalhandler.ACServerActive(5.0); //wait for the action server to come up

  int cnt=0;
 
  while(cnt<6) //select next goal
  {
	switch(cnt) 
	{
	  case 0: //start position
		{
		float position[]={0.0, 0.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, 0.72, 0.7};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
		}
	  break; 
	  case 1: //first station
	  {
		float position[]={-2.0, -1.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, -0.87, 0.5};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
	  }
	  break; 
	  case 2: //second station
	  {
		float position[]={-2.0, 1.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, 0.79, 0.61};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
	  }
	  break; 
      case 3: //third station
      {
		float position[]={4.0, 1.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, -0.43, 0.9};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
	  }
      break; 
      case 4: //fourth station
      {
		float position[]={4.5, -1.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={0,0,-0.62,0.78};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
	  }
      break; 
      case 5: //end position=start position
      {
		float position[]={0.0, 0.0, 0.0};//find out goal: set goal manually rviz; rostopic echo /move_base_simple/goal
		float orientation[]={ 0.0, 0.0, 0.72, 0.7};
		goalhandler.createNextGoal(position,orientation); //create goal to publish
	  }
      break;
    }
	
    goalhandler.ExecuteGoal();	
    cnt++; //count up for next position 
  }
  
  //system("$(rospack find soa_mmr3)/launch/kill.sh"); //if every position reached, kill all
  return 0;
}
