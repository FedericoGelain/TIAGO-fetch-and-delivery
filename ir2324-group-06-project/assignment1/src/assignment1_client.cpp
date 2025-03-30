#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment1/InfoAction.h>
#include <assignment1/InfoActionResult.h>

// Called every time feedback is received for the goal
void feedbackCb(const assignment1::InfoFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback from server: [ %s ]", feedback->status.c_str());
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "assignment1_client");
  // Create the action client
  actionlib::SimpleActionClient<assignment1::InfoAction> ac("info", true);

  //there are 2 ways to execute the code, either using the motion control law or not
  //if it isn't used, only 3 command line parameters are expected, which are the values for position and orientation of pose_B
  //if it is used, the additional x and y coordinate of the end of corridor are expected
  if(argc == 4) { //no motion control law
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait forever until the server is active

    //create the goal, initialize all the needed fields and send it to the action server
    ROS_INFO("Action server started, sending goal.");

    assignment1::InfoGoal goal;
    //assign to the goal the destination pose (x,y coordinates for the position, z is the yaw angle for the rotation)
    goal.pose_B_x = atof(argv[1]);
    goal.pose_B_y = atof(argv[2]);
    goal.pose_B_yaw = atof(argv[3]);
    goal.motion_control_law = false;
    ac.sendGoal(goal, actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleActiveCallback(), &feedbackCb);
  }
  else if(argc == 5) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait forever until the server is active

    //create the goal, initialize all the needed fields and send it to the action server
    ROS_INFO("Action server started, sending goal.");

    assignment1::InfoGoal goal;

    //assign to the goal the destination pose (x,y coordinates for the position, z is the yaw angle for the rotation) and the position
    //at the end of the corridor
    goal.pose_B_x = atof(argv[1]);
    goal.pose_B_y = atof(argv[2]);
    goal.pose_B_yaw = atof(argv[3]);
    goal.motion_control_law = atoi(argv[4]);

    ac.sendGoal(goal, actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleActiveCallback(), &feedbackCb);
  } else {
    ROS_INFO("Please specify the destination pose (Pose_B) and the control law flag!");
    return -1;
  }  
  
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();

    ROS_INFO("Action finished: %s",state.toString().c_str());

   //the action has successfully finished, so acquire the result obstacles positions and print them
    assignment1::InfoResult res = *ac.getResult();

	if(res.obstacles_positions_x.size() == 0) {
		ROS_INFO("No obstacled detected since the robot failed to reach its destination");
	}
	else {
    for (int i=0; i < res.obstacles_positions_x.size(); ++i)
      ROS_INFO("Obstacles Positions[%i]: (%f, %f)", i, res.obstacles_positions_x[i], res.obstacles_positions_y[i]);
	}
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  return 0;
}


