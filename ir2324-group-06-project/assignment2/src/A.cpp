#include "A.h"

void feedbackCb(const assignment1::InfoFeedbackConstPtr& feedback)
{
	ROS_INFO("Feedback from Info server: [ %s ]", feedback->status.c_str());
}

void feedbackCb_detection(const assignment2::DetectionFeedbackConstPtr& feedback)
{
 	ROS_INFO("Feedback from Detection server: [ %s ]", feedback->status.c_str());
}

void feedbackCb_manipulation(const assignment2::ManipulationFeedbackConstPtr& feedback)
{
 	ROS_INFO("Feedback from Manipulation server: [ %s ]", feedback->status.c_str());
}


NodeA::NodeA(std::string name, int extra) : ac("info", true), ac_detection("detection", true), ac_manipulation("manipulation", true) {
	client = n.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
	cmdvel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
	head_pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);

	initialize_waypoints_and_table_sides();

	service_id = -1;

	extra_part = extra;
}

cv::Vec3f NodeA::computeAverageColor(const cv::Mat& image) {
    cv::Vec3f avgColor(0, 0, 0);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            avgColor += image.at<cv::Vec3b>(i, j);
        }
    }

    // Calculate the average color
    avgColor /= static_cast<float>(image.rows * image.cols);

    return avgColor;
}

int NodeA::whereIsTheDesiredCylinder(cv::Vec3f COLOR_TO_PLACE_BGR, cv::Mat image) {
    cv::imwrite("assignment2/src/cylinders.jpg", image);
    
    int cols_third = image.cols / 3;

    // Create three separate images with sizes image.rows and image.cols/3
    cv::Mat leftImage(image.rows, cols_third, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat centerImage(image.rows, cols_third, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat rightImage(image.rows, cols_third, CV_8UC3, cv::Scalar(0, 0, 0));

    // Manually copy each pixel from the original image to the corresponding sector image
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < cols_third; ++j) {
            leftImage.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j);
            centerImage.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j + cols_third);
            rightImage.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(i, j + 2 * cols_third);
        }
    }

    std::vector<cv::Mat> sectorImages = {leftImage, centerImage, rightImage};

    float minDistance = std::numeric_limits<float>::max();
    int indexOfCylinder = -1;

    for (int sectorIndex = 0; sectorIndex < sectorImages.size(); ++sectorIndex) {
    	cv::Vec3f averageColor = computeAverageColor(sectorImages[sectorIndex]);
    	float distance = cv::norm(averageColor, COLOR_TO_PLACE_BGR);
    	if(distance < minDistance) {
    		minDistance = distance;
    		indexOfCylinder = sectorIndex;
    	}
    }
    
    cv::imwrite("assignment2/src/imageDesiredCylinder.jpg", sectorImages[indexOfCylinder]);
    
	// Define the padding width (in pixels)
	int paddingWidth = 10; // You can adjust this value as needed

	// Create matrices for padding
	cv::Mat paddingLeft(image.rows, paddingWidth, CV_8UC3, cv::Scalar(0, 0, 0)); // Black padding
	cv::Mat paddingCenter(image.rows, paddingWidth, CV_8UC3, cv::Scalar(0, 0, 0)); // Black padding

	// Concatenate the sector images with padding
	cv::Mat collage;
	cv::hconcat(leftImage, paddingLeft, collage);
	cv::hconcat(collage, centerImage, collage);
	cv::hconcat(collage, paddingCenter, collage);
	cv::hconcat(collage, rightImage, collage);

	// Save the collage image with padding
	cv::imwrite("assignment2/src/sectors.jpg", collage);

    return indexOfCylinder;
}

assignment1::InfoResult NodeA::goToPosition(actionlib::SimpleActionClient<assignment1::InfoAction>& ac, float x, float y, float yaw, bool control_law) {
	//create the goal, initialize all the needed fields and send it to the action server
    assignment1::InfoGoal goal;

    //assign to the goal the destination pose (x,y coordinates for the position, yaw is the angle that specifies the rotation) and
	//optionally set the flag to activate the motion control law
    goal.pose_B_x = x;
    goal.pose_B_y = y;
    goal.pose_B_yaw = yaw;

	goal.motion_control_law = control_law;

	//send the goal to the action server, specifying the feedback callback to keep active during the interaction with it
	//this callback will be used to print any feedback that the action server publishes to the client (regarding the robot state)
    ac.sendGoal(goal, actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment1::InfoAction>::SimpleActiveCallback(), &feedbackCb);
  
	//set the amount of time the action server has at most to complete the goal
  	bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

	if (finished_before_timeout)
	{
		//retrieve the state of the goal
		actionlib::SimpleClientGoalState state = ac.getState();

		//and check what is it. If it succeeded, it means that it correctly reached the destination (the table)
		//and you can proceed with the execution of the code
		if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
			ROS_INFO("Goal state: %s", state.toString().c_str());
		}
		else {//otherwise, it failed and so the execution stops
			ROS_ERROR("Goal state: %s", state.toString().c_str());
		}
	}
  	else {
		ROS_INFO("Action did not finish before the time out.");
	}
	return *ac.getResult();
}


bool NodeA::placeObject(tf::Vector3 cylinder_position, assignment2::ManipulationResult& res, int positionOfCylinder) {
	assignment2::ManipulationGoal goal_manipulation;

	// compute the transformation matrix from map to base_footprint to convert the cylinder position
    // in the correct frame used by moveIt
	try {
		tf_.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(2));
		tf_.lookupTransform("base_footprint", "map", ros::Time(0), transform);
	} catch (const tf::TransformException &ex) {
		ROS_ERROR("Failed to transform: %s", ex.what());
	}

    //convert the cylinder position into the base_footprint frame
	tf::Vector3 w(cylinder_position.x(), cylinder_position.y(), 0);
	w = transform * w;

    //additional height for z to place the robot arm above the cylinder
	const float SAFETY_HEIGHT = 0.2;

    //compute the object place position
	geometry_msgs::Pose obj_pose_to_place;
	obj_pose_to_place.position.x = w.x() + 0.75 * CYLINDERS_RADIUS;
	obj_pose_to_place.position.y = w.y();
	obj_pose_to_place.position.z = CYLINDERS_HEIGHT + SAFETY_HEIGHT;

	// Positioning fix based on the specific objects
	switch(positionOfCylinder) {
		case 0:
			obj_pose_to_place.position.y += 0.5 * CYLINDERS_RADIUS;
		break;
		case 2:
			obj_pose_to_place.position.y -= 0.5 * CYLINDERS_RADIUS;
		break;
	}

    //define the cylinder position
	geometry_msgs::Pose cylinder_pose;
	cylinder_pose.position.x = obj_pose_to_place.position.x;
    cylinder_pose.position.y = obj_pose_to_place.position.y;
    cylinder_pose.position.z = obj_pose_to_place.position.z;

    //fill the goal fields to send to the action server
	goal_manipulation.cylinder_pose = cylinder_pose;
	goal_manipulation.obj_pose_to_manipulate = obj_pose_to_place;
	goal_manipulation.obj_id = service_id;
	goal_manipulation.action = 1;

	ROS_INFO("Waiting for action server Manipulation to start.");
	ac_manipulation.waitForServer(); //will wait forever until the server is active

	ROS_INFO("Action server Manipulation started");

	ROS_INFO("Sending goal to Manipulation server..");
	ac_manipulation.sendGoal(goal_manipulation, actionlib::SimpleActionClient<assignment2::ManipulationAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment2::ManipulationAction>::SimpleActiveCallback(), &feedbackCb_manipulation);

	bool finished_before_timeout = ac_manipulation.waitForResult(ros::Duration(300.0));

	if (finished_before_timeout) {
        //check if the action server elaborated successfully the goal or not
		actionlib::SimpleClientGoalState state = ac_manipulation.getState();

        //if the state is aborted, it means that the place routine failed
        if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED) {
			ROS_ERROR("The place routine FAILED");
            res.success = false;
            return false;
		}

		//otherwise, it finished successfully
		res = *ac_manipulation.getResult();
        ROS_INFO("The place routine finished successfully");
        res.success = true;
		return true;
	} else {
		ROS_ERROR("Action Manipulation did not finish before the time out.");
        res.success = false;
		return false;
	}
}

void NodeA::initialize_waypoints_and_table_sides() {
	//set the end of the corridor as the first safe waypoint to reach the front side of the table and for the place part later
	waypoints.push_back(std::make_tuple(8.8, 0, -90));
    //second waypoint in the middle of the first side of the table
	waypoints.push_back(std::make_tuple(8, -1.8, -90));
	//third waypoint to approach the table from side 1 and 2
	waypoints.push_back(std::make_tuple(8.8, -2, -100));
	//fourth waypoint to approach the table from side 2 to 3
	waypoints.push_back(std::make_tuple(8.8, -4.2, 135));
	//fifth waypoint to approach the table from side 3 to 4
	waypoints.push_back(std::make_tuple(6.75, -4.2, 84));

	//front side of the table
	table_sides.push_back(std::make_tuple(7.55, -2, -83)); //front right, where the red cube is
	table_sides.push_back(std::make_tuple(8.1, -1.85, -100));//front left, where the blue hexagon is
	//left side of the table
	table_sides.push_back(std::make_tuple(8.8, -3, -187.5));
	//back side of the table, where the green triangle is
	table_sides.push_back(std::make_tuple(7.75, -4.0, 86));
	//right side of the table
	table_sides.push_back(std::make_tuple(6.65, -2.9, 10));
}

void NodeA::move_cmd_vel(float destination) {
	geometry_msgs::Twist msg;

    ros::Rate loop_rate(10); // Adjust the rate as needed

	ROS_INFO("Going forward -> cmd_vel");

    //until the robot has reached the position specified by the destination parameter
	while (true) {
        //retrieve the robot pose, to check where it currently is
		geometry_msgs::PoseWithCovarianceStampedConstPtr robotPose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", n);

        //if its y position is greater than the destination, it means that it has reached it
		if((*robotPose).pose.pose.position.y >= destination)
			break;

        //use the publisher to make the robot move straight
		msg.linear.x = 0.2;
		cmdvel_pub.publish(msg);
		loop_rate.sleep();
	}

	//the robot has reached its destination, so make it stop
	msg.linear.x = 0;
	cmdvel_pub.publish(msg);

	ROS_INFO("Stopped -> cmd_vel");
}

void NodeA::goingBackCmdVel(float destination) {
	geometry_msgs::Twist msg;

	ROS_INFO("Going back...");

	ros::Rate loop_rate(10); // Adjust the rate as needed

	// Set the desired rotation angle in radians
	double desired_angle = M_PI / 2.0; // 90 degrees

	// Specify the angular velocity for rotation
	double angular_velocity = 0.5; // Adjust the velocity as needed

	// Set the initial time
	ros::Time start_time = ros::Time::now();

	//ros::Rate loop_rate(10); // Adjust the rate as needed

	while (true) {
		// Calculate the elapsed time
		ros::Duration elapsed_time = ros::Time::now() - start_time;

		// Check if the desired angle is reached
		if (elapsed_time.toSec() >= (desired_angle / angular_velocity)) {
			// Stop the robot
			msg.angular.z = 0;
			cmdvel_pub.publish(msg);
			ROS_INFO("Robot reached the desired angle of %.2f degrees", desired_angle * 180.0 / M_PI);
			break; // Exit the loop
		} else {
			// Rotate the robot
			msg.angular.z = angular_velocity;
			cmdvel_pub.publish(msg);
		}

		loop_rate.sleep();
	}

    //the robot has turned correctly, so now it can move straight to the desired position
	move_cmd_vel(destination);
}


bool NodeA::findPlaceCylinder(tf::Vector3 &cylinder_position, int& positionOfCylinder) {
	
	ROS_INFO("Going back -> beginning");
	goToPosition(ac, std::get<0>(waypoints[0]), std::get<1>(waypoints[0]), std::get<2>(waypoints[0]), 0);

	//**************************************** MOVE TO CYLINDERS  *****************************************************************

    //move in front of the cylinder, and use res to retrieve the positions of the clusters that scan has detected
	assignment1::InfoResult res = goToPosition(ac, 11.4, 1.7, -90.0, 0);

    //reset the head orientation, if for some reason it wasn't already
	moveHead(0, 0);

	ROS_INFO("Clusters coordinates detected");

	std::vector<std::pair<float, float>> coordinates_without_outliers;

	//if the extra_part routine is chosen, then retrieve the coordinates from scan
	if(extra_part == 1) {
		ROS_INFO("Extra point routine. The initial coordinates of the cylinders are provided by the laser scan");
		//build the vector of coordinates of all clusters detected
		std::vector<std::pair<float, float>> coordinates;
		for(int i = 0; i < (int)res.obstacles_positions_x.size(); i++)
			coordinates.push_back(std::make_pair(res.obstacles_positions_x[i],res.obstacles_positions_y[i]));

		//remove the coordinates of outliers (the ones that aren't cylinders)
		//to do so, assuming that the robot is in front of the cylinders, we can do it knowing that the y coordinate
		//of the cylinders is restricted in a certain range (the center cylinder will have y almost 0, while the left and
		//right cylinder will have a y coordinate opposite to each other and not too large)

		for(int i = 0; i < coordinates.size(); i++) {
			if(std::abs(coordinates[i].first) < 2.0) {
				coordinates_without_outliers.push_back(std::make_pair(coordinates[i].first, coordinates[i].second));
			}
		}

		//sort the indices in descending order based on the y coordinate (the higher one corresponds to the left cylinder)
		std::sort(coordinates_without_outliers.begin(), coordinates_without_outliers.end(), [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
		   return a.second > b.second;
		});

		ROS_INFO("Coordinates of the cylinders without outliers");
		// check the elements of the coordinates remaining
		for(int i = 0; i < (int)coordinates_without_outliers.size(); i++) {
			ROS_INFO("X = %f Y = %f", coordinates_without_outliers[i].first, coordinates_without_outliers[i].second);
		}
		//if there are more or less than 3 coordinates
		if(coordinates_without_outliers.size() != 3) {
			ROS_ERROR("There aren't exactly 3 clusters, so more choices for the cylinders than expected");
			return false;
		}
	}
	else { //otherwise, define the hard-coded positions of the cylinders (already defined in the map frame)
		ROS_INFO("No extra point routine. The coordinates are already provided in the map frame");
		coordinates_without_outliers.push_back(std::make_pair(12.4, -0.2));
		coordinates_without_outliers.push_back(std::make_pair(11.5, -0.2));
		coordinates_without_outliers.push_back(std::make_pair(10.6, -0.2));
	}

    //move the robot head so that its camera can clearly see the 3 cylinders
	moveHead(-5.0 * M_PI/180, -20.0 * M_PI/180);

    //retrieve the image message
	sensor_msgs::ImageConstPtr img_msg;
	img_msg = ros::topic::waitForMessage<sensor_msgs::Image>(IMAGE_TOPIC, n);

	positionOfCylinder = -1;

	// Process the received image message
	if (img_msg) {
		//retrieve the image in BGR8 format
		cv::Mat image = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

        //based on the id of the object that has to be placed, call the function with the correct color to find in the image
		switch(service_id) {
			case 1:
				positionOfCylinder = whereIsTheDesiredCylinder(cv::Vec3f(255.0, 0.0, 0.0), image);
			break;

			case 2:
				positionOfCylinder = whereIsTheDesiredCylinder(cv::Vec3f(0.0, 255.0, 0.0), image);
			break;

			case 3:
				positionOfCylinder = whereIsTheDesiredCylinder(cv::Vec3f(0.0, 0.0, 255.0), image);
			break;
		}
	}
	else {
		ROS_ERROR("No image received.");
        return false;
	}

    //reset the orientation of the head
	moveHead(0, 0);

    //if the position of the cylinder wasn't found, return false and try again
	if(positionOfCylinder == -1) {
		ROS_ERROR("The scan failed.. positionOfCylinder %d.. Trying again..", positionOfCylinder);
		return false;
	}

	cylinder_position = tf::Vector3(coordinates_without_outliers[positionOfCylinder].first, coordinates_without_outliers[positionOfCylinder].second, 0);

	if(extra_part == 1) {
		//the coordinates returned by scan are expressed in the base_laser_link frame, so to make the robot move into the correct position they have to be converted into map
		ROS_INFO("Cylinder found. Its coordinates in the base_laser_link frame are at %f, %f", coordinates_without_outliers[positionOfCylinder].first, coordinates_without_outliers[positionOfCylinder].second);

		try {
			tf_.waitForTransform("map", "base_laser_link", ros::Time(0), ros::Duration(2));
			tf_.lookupTransform("map", "base_laser_link", ros::Time(0), transform);
		} catch (const tf::TransformException &ex) {
			ROS_ERROR("Failed to transform: %s", ex.what());
		}
		
		cylinder_position = transform * cylinder_position;
	}

	ROS_INFO("Cylinder coordinates in map frame are at %f, %f", cylinder_position.x(), cylinder_position.y());

	return true;
}

void NodeA::moveHead(double roll, double pitch) {
	try {
		trajectory_msgs::JointTrajectory trajectory; //define the trajectory to move the two joints of the head
		trajectory.joint_names = {"head_1_joint", "head_2_joint"};

		//set the desired position and velocity of the point that the head has to reach
		trajectory_msgs::JointTrajectoryPoint point;
		point.positions = {roll, pitch};
		point.velocities = {0.0, 0.0};
		point.time_from_start = ros::Duration(0.5); // adjust duration as needed

		trajectory.points.push_back(point);

		//publish the trajectory in order to perform it
		head_pub.publish(trajectory);

		ros::Duration(1.0).sleep(); // wait for the head to be in the correct position
		ROS_INFO("Head rotated to a roll angle %f and a pitch angle of %f degrees", roll, pitch);
	}
	catch (ros::Exception& e) {
		ROS_ERROR("ROS Exception: %s", e.what());
	}
}

void NodeA::start() {
	//set the request fields of the request
	srv.request.ready = true;
	srv.request.all_objs = true;

	if (!client.call(srv)) {
		//for some reason the service call failed, so interrupt the execution
		ROS_ERROR("Failed to call service");
		return;
	}

	//the call succeeded, so the response field of the service has been set
	//in this case, it contains the ID of the object that the robot has to pick and place
	for(int srv_i = 0; srv_i < srv.response.ids.size(); srv_i++) {
		ROS_INFO("Id of the current object to pick: %d", srv.response.ids[srv_i]);
		service_id = srv.response.ids[srv_i];

		ROS_INFO("Waiting for action server Assignment1 to start.");
		ac.waitForServer(); //now wait until the server starts (indefinite amount of time)

		//create the goal, initialize all the needed fields and send it to the action server
		ROS_INFO("Action server Assignment1 started");

		geometry_msgs::Twist msg;

		//the loop will move around all sides of the table, so that you can locate all the objects and create the collisions
		for(int i = 0; i < waypoints.size(); i++) {

			ROS_INFO("Sending goals to action server assignment1");
			ROS_INFO("Moving to waypoint %d", i);

			//first move to the waypoint
			goToPosition(ac, std::get<0>(waypoints[i]), std::get<1>(waypoints[i]), std::get<2>(waypoints[i]), 0);

			if(i == waypoints.size()-1)
				move_cmd_vel(std::get<1>(table_sides[i]) - 0.3);

			ROS_INFO("Moving to table side %d", i);

			//and then to the table side
			goToPosition(ac, std::get<0>(table_sides[i]), std::get<1>(table_sides[i]), std::get<2>(table_sides[i]), 0);

			ROS_INFO("Waiting for action server Detection to start.");
			ac_detection.waitForServer(); //will wait forever until the server is active

			ROS_INFO("Action server Detection started");
			//create the goal, initialize all the needed fields and send it to the action server
			ROS_INFO("Sending goal to action server Detection");

			// the goal is simply to perform the detection
			assignment2::DetectionGoal goal_detection;
			goal_detection.service_id = service_id;

			//rotate the head correctly before starting the detection of the markers
			moveHead(0, -35.0 * M_PI / 180);

			ac_detection.sendGoal(goal_detection, actionlib::SimpleActionClient<assignment2::DetectionAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment2::DetectionAction>::SimpleActiveCallback(), &feedbackCb_detection);

			bool finished_before_timeout_detection = ac_detection.waitForResult(ros::Duration(90.0));
			if (finished_before_timeout_detection) {
				//the action has successfully finished, so acquire the result obstacles positions and print them
				assignment2::DetectionResult res_detection = *ac_detection.getResult();

				actionlib::SimpleClientGoalState state = ac_detection.getState();

				if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED) {
					ROS_ERROR("The detection FAILED, moving to the next side of the table");
					continue;
				}

				assignment2::ManipulationGoal goal_manipulation;

				goal_manipulation.detections_poses = res_detection.detections_poses;
				goal_manipulation.detections_ids = res_detection.detections_ids;
				goal_manipulation.detections_qr_size = res_detection.detections_qr_size;

				if(res_detection.object_found == 1) {
					ROS_INFO("Object ID %d found. Initializing the goal to pick it", service_id);

					goal_manipulation.obj_pose_to_manipulate = res_detection.obj_pose_to_pick;
					goal_manipulation.obj_id = res_detection.obj_id;
					goal_manipulation.qr_size = res_detection.qr_size;
					goal_manipulation.action = 0; // 0 -> pick
				}
				else {
					ROS_INFO("Object ID %d NOT found", service_id);
					goal_manipulation.action = -1; // only generate collisions
				}

				ROS_INFO("Waiting for action server Manipulation to start.");
				ac_manipulation.waitForServer(); //will wait forever until the server is active

				ROS_INFO("Action server Manipulation started");

				ROS_INFO("Sending goal to Manipulation server..");
				ac_manipulation.sendGoal(goal_manipulation, actionlib::SimpleActionClient<assignment2::ManipulationAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<assignment2::ManipulationAction>::SimpleActiveCallback(), &feedbackCb_manipulation);

				bool finished_before_timeout_manipulation = ac_manipulation.waitForResult(ros::Duration(90.0));

				if (finished_before_timeout_manipulation) {
					actionlib::SimpleClientGoalState state = ac_manipulation.getState();

                    if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED) {
					    ROS_ERROR("The manipulation FAILED");

						//move the head back to its original state
						moveHead(0, 0);

						continue;
				    }

					ROS_INFO("Action server Manipulation finished: %s",state.toString().c_str());

					//the action has successfully finished, so acquire the result obstacles positions and print them
					assignment2::ManipulationResult res = *ac_manipulation.getResult();

					if(goal_manipulation.action == 0) {
						if(i == waypoints.size() - 2) {
							goToPosition(ac, std::get<0>(waypoints[4]), std::get<1>(waypoints[4]), std::get<2>(waypoints[4]), 0);
							move_cmd_vel(-2.2);
						} else if (i == waypoints.size() - 1) {
							goingBackCmdVel(-2.2);
						} else {
							for (int k = i; k >= 1; k--) {
								goToPosition(ac, std::get<0>(waypoints[k]), std::get<1>(waypoints[k]), std::get<2>(waypoints[k]), 0);
							}
						}

						int positionOfCylinder;
						tf::Vector3 cylinder_position;
						while(!findPlaceCylinder(cylinder_position, positionOfCylinder));

						goToPosition(ac, cylinder_position.x(), cylinder_position.y()+0.5, -90, 0);

						assignment2::ManipulationResult res;
						placeObject(cylinder_position, res, positionOfCylinder);

						i = waypoints.size();
					}
					else if (i == waypoints.size() - 1) {
						goingBackCmdVel(-2.2);
						goToPosition(ac, std::get<0>(waypoints[0]), std::get<1>(waypoints[0]), std::get<2>(waypoints[0]), 0);
					}
				} else {
					ROS_ERROR("Action Manipulation did not finish before the time out.");
					return;
				}

				//move the head back to its original state
				moveHead(0, 0);
			}
			else {
				ROS_ERROR("Action Detection did not finish before the time out.");
				return;
			}
		}
	}
}


int main(int argc, char **argv)
{
	//initialize the node as A
	ros::init(argc, argv, "A");

	//if the extra_point flag is not specified, run the routine in the standard way
	if(argc < 2) {
		NodeA nodeA("a", 0);
		nodeA.start();
	}
	else if (argc == 2) { //otherwise, run it in the mode specified by the user
		int extra = atoi(argv[0]);
		NodeA nodeA("a", extra);
		nodeA.start();
	}
	else {
		ROS_FATAL("Number of command line parameters is wrong");
		return -1;
	}

	return 0;
}
