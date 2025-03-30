#include "C.h"

ManipulationAction::ManipulationAction(std::string name) :
      as_(nh_, name, boost::bind(&ManipulationAction::executeCB, this, _1), false),
      action_name_(name), move_group_interface_arm("arm"), move_group_interface_armtorso("arm_torso"){

	as_.start();

	// Configuration of move group interfaces
	move_group_interface_arm.setPoseReferenceFrame("base_footprint");
	move_group_interface_arm.setPlannerId("SBLkConfigDefault");
	move_group_interface_arm.setPlanningTime(30.0);

	move_group_interface_armtorso.setPoseReferenceFrame("base_footprint");
	move_group_interface_armtorso.setPlannerId("SBLkConfigDefault");
	move_group_interface_armtorso.setPlanningTime(30.0);

	// Get the interested joints of arm move group interface
	const robot_state::JointModelGroup* joint_model_group = move_group_interface_arm.getCurrentState()->getJointModelGroup("arm");
	
	// Get current state (joints positions)
	moveit::core::RobotStatePtr current_state = move_group_interface_arm.getCurrentState();

	// Copy the joints positions into the 'joint_tucked_arm_positions' vector (secure pose of the arm)
	current_state->copyJointGroupPositions(joint_model_group, joint_tucked_arm_positions);
	for(int i = 0; i < joint_tucked_arm_positions.size(); i++) {
		// Round the positions to two decimal places
		joint_tucked_arm_positions[i] = std::ceil(joint_tucked_arm_positions[i] * 100.0) / 100.0;
	}

	// Fill the vector with the joints positions of the intermediate pose
	joint_extended_arm_positions.push_back(0.07);
	joint_extended_arm_positions.push_back(0.26);
	joint_extended_arm_positions.push_back(-2.22);
	joint_extended_arm_positions.push_back(-0.19);
	joint_extended_arm_positions.push_back(0.46);
	joint_extended_arm_positions.push_back(-0.07);
	joint_extended_arm_positions.push_back(-0.92);

	client_detach = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
	client_attach = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

	// Try to create the gripper controller client
	if(!createControllerClient(GripperClient, "gripper_controller")) {
		sendFeedback("Failed to create the controller client (gripper_controller)");
	}

	// Try to reset the arm pose
	ROS_INFO("Resetting arm..");
	if(!resetArmPose(-1))
		sendFeedback("Failed to reset the arm pose during the startup routine");

	// Try to open the gripper 
	ROS_INFO("Resetting gripper..");
	if(!moveGripper(0.04))
		sendFeedback("Failed to close the gripper during the startup routine");
}

void ManipulationAction::sendFeedback(std::string feed) {
	feedback_.status.clear();
	feedback_.status.assign(feed);
	as_.publishFeedback(feedback_);
}

bool ManipulationAction::createControllerClient(follow_trajectory_Ptr& actionClient, const std::string& controller_name)
{
    ROS_INFO("Creating action client to %s ...", controller_name.c_str());

    std::string name = "/"+controller_name+"/follow_joint_trajectory";
    actionClient.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(name));

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations) {
        ROS_DEBUG("Waiting for the %s action server to come up", controller_name.c_str());
        ++iterations;
    }

    if (iterations == max_iterations) {
        ROS_ERROR("Error in createControllerClient: %s action server not available", controller_name.c_str());
        return false;
    } else {
        ROS_INFO("%s action server is available!", controller_name.c_str());
        return true;
    }
}

bool ManipulationAction::placeObject(const assignment2::ManipulationGoalConstPtr &goal) {
    geometry_msgs::Pose target_pose = goal->obj_pose_to_manipulate;

	tf2::Quaternion orientation;

	std::vector<double> intermediate_joint_place_positions = joint_extended_arm_positions;
	intermediate_joint_place_positions[3] = 0.98;
	intermediate_joint_place_positions[6] = 0;

	switch(goal->obj_id) {
		case 1:
			orientation.setRPY(M_PI/2.0, 0.0, 0.0); // Roll, Pitch, Yaw
			target_pose.position.x -= 0.2; 
			break;
		case 2: case 3: 
			performJointsMotion(intermediate_joint_place_positions);
			orientation.setRPY(M_PI/2.0, M_PI/2.0, 0.0); // Roll, Pitch, Yaw
			target_pose.position.z += 0.1; 
			break;
	}

    target_pose.orientation = tf2::toMsg(orientation);

	// Try to move arm on top of cylinder 
    ROS_INFO("Moving arm on top of cylinder..");
    if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
    	sendFeedback("Failed to move the arm on top of cylinder to place object " + std::to_string(goal->obj_id));
    	return false;
    }

	// Try to reset the gripper
    ROS_INFO("Resetting gripper..");
    if(!moveGripper(0.04)) {
		sendFeedback("Failed to open the gripper during the place routine");
		return false;
	}

	// Try to drop the object 
    ROS_INFO("Dropping object..");
    if(!detachObject(goal->obj_id)) {
    	sendFeedback("Failed to dropping the object during the place routine");
    	return false;
    }

	if(goal->obj_id != 1) {
		performJointsMotion(intermediate_joint_place_positions);
	}

	// Try to reset the arm pose 
    ROS_INFO("Resetting arm pose..");
    if(!resetArmPose(-1)) {
		sendFeedback("Failed to reset the arm pose during the place routine");
		return false;
	}
    return true;
}

void ManipulationAction::generateCylinderCollision(geometry_msgs::Pose cylinder_pose) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject currObj;

    currObj.header.frame_id =  move_group_interface_armtorso.getPoseReferenceFrame();

	std::string cylinder_id("cylinder");
    currObj.id = cylinder_id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);

	const double SIZE_INCREMENT = 1.2;

    primitive.dimensions[0] = CYLINDERS_HEIGHT;
    primitive.dimensions[1] = CYLINDERS_RADIUS * SIZE_INCREMENT;

	cylinder_pose.position.z = primitive.dimensions[0] / 2.0;
    currObj.primitives.push_back(primitive);
    currObj.primitive_poses.push_back(cylinder_pose);

    currObj.operation = currObj.ADD;
    collision_objects.push_back(currObj);

	planning_scene_interface.applyCollisionObjects(collision_objects);

	ROS_INFO("Added all collisions");
}

bool ManipulationAction::detachObject(int obj_id) {
    gazebo_ros_link_attacher::Attach srv_detach;

	// Choose the right frames and links to detach
    switch(obj_id) {
		case 1:
			srv_detach.request.model_name_1 = "Hexagon";
			srv_detach.request.link_name_1 = "Hexagon_link";
		break;

		case 2:
			srv_detach.request.model_name_1 = "Triangle";
			srv_detach.request.link_name_1 = "Triangle_link";
		break;

		case 3:
			srv_detach.request.model_name_1 = "cube";
			srv_detach.request.link_name_1 = "cube_link";
		break;
    }

    srv_detach.request.model_name_2 = "tiago";
    srv_detach.request.link_name_2 = "arm_7_link";

	// Call the detaching service
    if (client_detach.call(srv_detach)) {
    	ROS_INFO("Detached successfully");
    	return true;
    } else {
    	ROS_ERROR("Failed to detach");
    	return false;
    }
}

bool ManipulationAction::performJointsMotion(std::vector<double> joints_values) {
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	move_group_interface_arm.setStartState(*move_group_interface_arm.getCurrentState());
	move_group_interface_arm.setJointValueTarget(joints_values);

	bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	if(success) {
		// Execute the motion
		move_group_interface_arm.move();
		ros::Duration(2.0).sleep();
		ROS_INFO("Arm moved successfully");
		return true;
	}
	else {
		ROS_ERROR("Failed to plan and execute arm movement");
		return false;
	}
}

bool ManipulationAction::resetArmPose(int obj_id = -1) {
	// If TIAGO gets the blue object or it finishes the place of the object, it directly reaches the secure arm pose.
	// If TIAGO gets the red or green object, it performs the needed arm movements to reach first the intermediate arm pose
	// and then the secure arm pose.
	if(obj_id != -1 && obj_id != 1)
		performJointsMotion(joint_extended_arm_positions);

	return performJointsMotion(joint_tucked_arm_positions);
}

bool ManipulationAction::attachObject(int obj_id) {
	gazebo_ros_link_attacher::Attach srv_attach;

	// Choose the right frames and links to attach
	switch(obj_id) {
		case 1:
			srv_attach.request.model_name_1 = "Hexagon";
			srv_attach.request.link_name_1 = "Hexagon_link";
		break;

		case 2:
			srv_attach.request.model_name_1 = "Triangle";
			srv_attach.request.link_name_1 = "Triangle_link";
		break;

		case 3:
			srv_attach.request.model_name_1 = "cube";
			srv_attach.request.link_name_1 = "cube_link";
		break;
	}

	srv_attach.request.model_name_2 = "tiago";
	srv_attach.request.link_name_2 = "arm_7_link";

	// Call the attaching service
	if (client_attach.call(srv_attach)) {
		ROS_INFO("Attached successfully");
		return true;
	} else {
		ROS_ERROR("Failed to attach");
		return false;
	}
}

bool ManipulationAction::moveGripper(float joints_value) {
	// Check if the action client is connected
    if (!GripperClient->isServerConnected()) {
        ROS_ERROR("Arm action server is not connected");
        return false;
    }

    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal goal;

    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = joints_value;
    goal.trajectory.points[0].positions[1] = joints_value;

    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    for (int j = 0; j < 2; ++j) {
        goal.trajectory.points[0].velocities[j] = 0.0;
    }

    // To be reached 4 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

    // Sends the command to start the given trajectory 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    GripperClient->sendGoal(goal);

    // Wait for trajectory execution
    while (!(GripperClient->getState().isDone()) && ros::ok()) {
        ros::Duration(0.1).sleep(); // sleep for 0.1 seconds
    }

    return true;
}

bool ManipulationAction::movePlanningGroup(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface& move_group) {
    // Sets the current state of the robot
	move_group.setStartState(*move_group.getCurrentState());

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Set target pose
    move_group.setPoseTarget(target_pose);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success) {
		// Execute the motion
		move_group.move();
		ros::Duration(2.0).sleep();
		ROS_INFO("Arm moved successfully");
		return true;
    }
    else {
    	ROS_ERROR("Failed to plan and execute arm movement");
    	return false;
    }
}

bool ManipulationAction::pickObject(const assignment2::ManipulationGoalConstPtr &goal, float& pick_height) {
	if(goal->obj_id == 2 || goal->obj_id == 3) {
		if(!performJointsMotion(joint_extended_arm_positions)) {
			ROS_ERROR("Failed to set the intermediate pose of the arm");
			sendFeedback("Failed to set the intermediate pose of the arm");
			return false;	
		}
	}

	// Set the target pose for picking up the object
	geometry_msgs::Pose starting_pose = goal->obj_pose_to_manipulate;	
	geometry_msgs::Pose target_pose = goal->obj_pose_to_manipulate;

	tf2::Quaternion orientation;
	
	float HEIGHT_APPROACH_OBJECT, HEIGHT_ASCENT;

	// Set the desired orientation to align with -y axis
	switch(goal->obj_id) {
		case 1: 
			HEIGHT_APPROACH_OBJECT = 0.5;			
			orientation.setRPY(M_PI/2.0, 0.0, 0.0); // Roll, Pitch, Yaw
		
			target_pose.position.x -= (0.2 + 0.1);

			
			HEIGHT_ASCENT = pick_height/2.0;
			break;
		case 2: case 3: 
			orientation.setRPY(M_PI/2.0, M_PI/2.0, 0.0); // Roll, Pitch, Yaw
			HEIGHT_APPROACH_OBJECT = 0.3;
			HEIGHT_ASCENT = pick_height + 0.2;
			break;
	}

	target_pose.orientation = tf2::toMsg(orientation);
			
	// Move arm on top of the object
	ROS_INFO("Moving arm on top of object..");
	target_pose.position.z += HEIGHT_APPROACH_OBJECT;
	if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
		sendFeedback("Failed to move the arm on top of object " + std::to_string(goal->obj_id) + " during the pick routine");
		return false;
	}
	
	// Move arm on the object
	ROS_INFO("Moving arm on the object..");
	const float HEIGHT_OFFSET = (TABLE_HEIGHT + TABLE_THICKNESS + HEIGHT_ASCENT);
	target_pose.position.z = HEIGHT_OFFSET;
	if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
		sendFeedback("Failed to move the arm on the object " + std::to_string(goal->obj_id) + " during the pick routine");
		return false;
	}

	// If TIAGO wants to get the blue object then we move the arm a bit forward 
	if(goal->obj_id == 1) {
		target_pose.position.x += 0.1;

		if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
			sendFeedback("Failed to move the arm on the object " + std::to_string(goal->obj_id) + " (cylinder case) during the pick routine");
			return false;
		}
	}

	// Closing the gripper
	ROS_INFO("Closing gripper..");
	if(!moveGripper(0)) {
		sendFeedback("Failed to close the gripper during the pick routine");
		return false;
	}

	// Removing object collision
	ROS_INFO("Removing Object ID %d collision..", goal->obj_id);
    std::vector < std::string> objects_ids;
    objects_ids.push_back(std::to_string(goal->obj_id));
    planning_scene_interface.removeCollisionObjects(objects_ids);
    ros::Duration(2.0).sleep(); // wait
    ROS_INFO("Collision successfully removed");

	// Attach the object to the gripper
    ROS_INFO("Attaching to the gripper");
	if(!attachObject(goal->obj_id)) {
		sendFeedback("Failed to attach the object to the gripper during the pick routine");
		return false;
	}
    ROS_INFO("Object attached to the gripper");
 	
	// Raising arm
	ROS_INFO("Raising arm..");
	if(goal->obj_id == 1) {
		target_pose.position.x -= 0.1;

		if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
			sendFeedback("Failed to raise the arm (cylinder case) during the pick routine");
			return false;
		}
	}
	target_pose.position.z = starting_pose.position.z;
	target_pose.position.z += HEIGHT_APPROACH_OBJECT;
	if(!movePlanningGroup(target_pose, move_group_interface_armtorso)) {
		sendFeedback("Failed to raise the arm during the pick routine");
		return false;
	}
	
	// Resetting arm pose
	ROS_INFO("Resetting arm pose..");
	if(!resetArmPose(goal->obj_id)) {
		sendFeedback("Failed to reset the arm pose during the pick routine");
		return false;
	}
	return true;
}

void ManipulationAction::generateCollisions(const assignment2::ManipulationGoalConstPtr &goal, float& pick_height) {
    // Stores and organizes useful variables from the goal 
	int num_tags = (goal->detections_poses).size();
    int pick_id = goal->obj_id;

    std::vector<geometry_msgs::Pose> tag_poses (goal->detections_poses);
    std::vector<int> tag_ids (goal->detections_ids);
    std::vector<double> tag_size (goal->detections_qr_size);

    // Vector of collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;

	// For all found objects
    for(int i = 0; i < num_tags; i++) {
        moveit_msgs::CollisionObject currObj;

        currObj.header.frame_id =  move_group_interface_armtorso.getPoseReferenceFrame();

	    currObj.id = std::to_string(tag_ids[i]);

        shape_msgs::SolidPrimitive primitive;

        ROS_INFO("Req-C: %f, %d, %f", tag_poses[i].position.z, tag_ids[i], tag_size[i]);

        float SIZE_PERC_INCREASING = 1.5; // To increase the dimension of the collisions 

        switch(tag_ids[i]) {
            case 2: // Triangle
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = tag_size[i]*SIZE_PERC_INCREASING;
                primitive.dimensions[1] = tag_size[i]*SIZE_PERC_INCREASING;
                primitive.dimensions[2] = tag_size[i]*SIZE_PERC_INCREASING;

				if(pick_id == tag_ids[i])
	                pick_height = primitive.dimensions[2]; 

                break;

            case 3: // Cube
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = tag_size[i]*SIZE_PERC_INCREASING;
                primitive.dimensions[1] = tag_size[i]*SIZE_PERC_INCREASING;
                primitive.dimensions[2] = tag_size[i]*SIZE_PERC_INCREASING;

				if(pick_id == tag_ids[i])
	                pick_height = primitive.dimensions[2];

                break;

            default: // Hexagon and others
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[0] = tag_poses[i].position.z - TABLE_HEIGHT + TABLE_THICKNESS;   // qr z - table z + spessore tavolo
                ROS_INFO("Height cyl / hex: %f", primitive.dimensions[0]);
                primitive.dimensions[1] = tag_size[i]*std::sqrt(2);

				if(pick_id == tag_ids[i])
	                pick_height = primitive.dimensions[0];

				//to avoid inconsistencies with how the marker on top of the cylinders,
				//the orientation is set to 0 to create straight collisions
				tf2::Quaternion quaternion;
			    quaternion.setRPY(0, 0, 0);

				tag_poses[i].orientation.x = quaternion.x();
				tag_poses[i].orientation.y = quaternion.y();
				tag_poses[i].orientation.z = quaternion.z();
				tag_poses[i].orientation.w = quaternion.w();

                break;
        }

        currObj.primitives.push_back(primitive);

	    tag_poses[i].position.z = TABLE_HEIGHT + TABLE_THICKNESS + primitive.dimensions[0]/2; // Table height + Table thickness + Half height of the object 

	    currObj.primitive_poses.push_back(tag_poses[i]);
	    currObj.operation = currObj.ADD; // To add the object 
        collision_objects.push_back(currObj);
    }

	planning_scene_interface.applyCollisionObjects(collision_objects);
	
	ROS_INFO("Added all collisions");
}

void ManipulationAction::buildTable() {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_armtorso.getPoseReferenceFrame();
	 
    std::string table_id("table");
    collision_object.id = table_id;
	 
    shape_msgs::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.96; // real value: 0.913
    primitive.dimensions[1] = 0.96; // real value: 0.913
    primitive.dimensions[2] = TABLE_THICKNESS;

    collision_object.primitives.push_back(primitive);
	
    geometry_msgs::Pose pose;
    pose.position.x = 7.799619;
    pose.position.y = -2.975654;
    pose.position.z = TABLE_HEIGHT + primitive.dimensions[2]/2;

    // After defining the TransformListener, we wait until the transformation between the two frames is available (we want to retrieve the most recent one).
    // The frame of the detections is the same for all objects and can be accessed from the header of the pose.
	try {
		tf_.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(2));
  	    tf_.lookupTransform("base_footprint", "map", ros::Time(0), transform); // Save into transform the transformation between the two frames
	} catch (const tf::TransformException &ex) {
		ROS_ERROR("Failed to transform: %s", ex.what());
	}

	tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
    
	v = transform * v;
    
    // Convert the orientation of the pose
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    q = transform * q;

    // Create the converted pose and return it
    geometry_msgs::Pose converted_pose;

    converted_pose.position.x = v.x();
    converted_pose.position.y = v.y();
    converted_pose.position.z = v.z();
    converted_pose.orientation.x = q.x();
    converted_pose.orientation.y = q.y();
    converted_pose.orientation.z = q.z();
    converted_pose.orientation.w = q.w();

    collision_object.primitive_poses.push_back(converted_pose);
    collision_object.operation = collision_object.ADD;
	 
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
	 
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void ManipulationAction::executeCB(const assignment2::ManipulationGoalConstPtr &goal) { 
  
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      return;
    }

	buildTable();

	float pick_height = 0.0;

    // Add objects collisions
    if(goal->action != 1)
        generateCollisions(goal, pick_height);

    // Pick object
    if(goal->action == 0) {
    	if(!pickObject(goal, pick_height)) {
			ROS_ERROR("Failed to pick the object %d", goal->obj_id);
			sendFeedback("Failed to pick the object " + std::to_string(goal->obj_id)); 
			resetArmPose(-1);
			as_.setAborted(result_);
			return;
		}
    }
    // Place object
    if (goal->action == 1) {
		generateCylinderCollision(goal->cylinder_pose);
    	if(!placeObject(goal)) {
			ROS_ERROR("Failed to place the object %d", goal->obj_id);
			sendFeedback("Failed to place the object " + std::to_string(goal->obj_id)); 
			resetArmPose(-1);
			as_.setAborted(result_);
			return;
		}
	}
    	
    as_.setSucceeded(result_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "C");
    
    ros::NodeHandle nh;
   
   	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner beforehand.
	ros::AsyncSpinner spinner(1);
	spinner.start();

    // Manipulation
    ManipulationAction manipulation("manipulation");

    ros::waitForShutdown();
  
    return 0;
}
