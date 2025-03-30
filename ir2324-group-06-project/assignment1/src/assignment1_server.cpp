#include "assignment1.h"

// Action specification for move_base

double InfoAction::euclideanDistance(const InfoAction::Point& point1, const InfoAction::Point& point2) {
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void InfoAction::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, bool& end_corridor) {
	ROS_INFO("Pos: %f", msg->pose.pose.position.x);
	
	if(msg->pose.pose.position.x > 7.0 && msg->pose.pose.position.x < 8.0)
		end_corridor = true;
	else
		end_corridor = false;
}

std::vector<int> InfoAction::computeNeighbours(const Point& center, const std::vector<Point>& points, double epsilon) {
	std::vector<int> neighbors;

	for (int i = 0; i < points.size(); ++i) {
	    // if the euclidean distance is below the chosen threshold, add the point as a neighbour of currentPoint
	   	// currentPoint is considered as a neighbour since the euclidean distance with itself is 0
	    if (euclideanDistance(center, points[i]) < epsilon) {
	       neighbors.push_back(i);
	    }
	}

	return neighbors;
}
		
double InfoAction::calculateClusterVariance(const std::vector<InfoAction::Point>& cluster) {
    if (cluster.empty()) {
		return 0.0;
    }

    double meanX = 0.0, meanY = 0.0;

    // Calculate mean values
    for (const auto& point : cluster) {
		meanX += point.x;
		meanY += point.y;
    }

    meanX /= cluster.size();
    meanY /= cluster.size();

    double variance = 0.0;

    // Calculate variance
    for (const auto& point : cluster) {
		variance += pow(point.x - meanX, 2) + pow(point.y - meanY, 2);
    }

    return variance / cluster.size();
}
	
std::vector<std::vector<InfoAction::Point>> InfoAction::buildClusters(const std::vector<InfoAction::Point>& points) {
	// Map cluster IDs to vectors of Point
	std::map<int, std::vector<InfoAction::Point>> clusters;

	// Iterate through points and group them by clusterId
	for (const InfoAction::Point& point : points) {
		if (point.clusterId != -1) {
		    clusters[point.clusterId].push_back(point);
		}
	}
	    
	// Convert the map values to a vector of vectors
	std::vector<std::vector<InfoAction::Point>> result;
	result.reserve(clusters.size());
	
	for (auto& pair : clusters) {
		result.push_back(std::move(pair.second));
	}

	return result;
}
	
void InfoAction::plotClusters(const std::vector<std::vector<InfoAction::Point>>& clusters, const cv::Size& imageSize, const std::string& filename) {
	// Create a black canvas
	cv::Mat image(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

	// Find minimum and maximum coordinates
	double minX = std::numeric_limits<double>::max();
	double minY = std::numeric_limits<double>::max();
	double maxX = std::numeric_limits<double>::lowest();
	double maxY = std::numeric_limits<double>::lowest();

	for (const auto& cluster : clusters) {
		for (const InfoAction::Point& point : cluster) {
			minX = std::min(minX, point.x);
			minY = std::min(minY, point.y);
			maxX = std::max(maxX, point.x);
			maxY = std::max(maxY, point.y);
		}
	}

	// Normalize points to fit within the imageSize
	double scaleFactorX = static_cast<double>(imageSize.width) / (maxX - minX);
	double scaleFactorY = static_cast<double>(imageSize.height) / (maxY - minY);
	
	// Draw each cluster with corresponding color
	for (size_t i = 0; i < clusters.size(); ++i) {
		//cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);

		for (const InfoAction::Point& point : clusters[i]) {
			// Normalize coordinates
			int normalizedX = static_cast<int>((point.x - minX) * scaleFactorX);
			int normalizedY = static_cast<int>((point.y - minY) * scaleFactorY);

			// Draw point on the image
			cv::circle(image, cv::Point(normalizedX, normalizedY), 5, colorMap[i], -1);
		}
	}

	// Save the image
	cv::flip(image, image, 1);  // 1 indicates flipping around the y-axis. Needed to display consistently with gazebo
	cv::imwrite(filename, image);
}
	
std::vector<int> InfoAction::obstaclesDetectionThresholding(const std::vector<double>& variances, const double& threshold) {
	// Find the minimum value
	double minValue = *std::min_element(variances.begin(), variances.end());

	std::vector<int> obstaclesIndices;

	int j = 0;

	// Iterate through the vector and save indices of values within the threshold
	for (int i = 0; i < variances.size(); ++i) {
		if (variances[i] == minValue || std::abs(variances[i] - minValue) <= threshold) {
			if(variances[i] > 0) {
			ROS_INFO("Obstacle %d: cluster %d, %s", ++j, i, colorName[i].c_str());
			obstaclesIndices.push_back(i);
			}
		}
	}
	return obstaclesIndices;
}
		 
void InfoAction::scanCallback(const sensor_msgs::LaserScanConstPtr& msg, assignment1::InfoResult& res)
{
	std::vector<float> obstacles_x;
	std::vector<float> obstacles_y;
	
	// compute the size of the array of data coming from the laser scan as (angle_max - angle_mix) / angle_increment.
	int size = (msg->angle_max-msg->angle_min) / msg->angle_increment;
	
	// vector in which all points converted into cartesian coordinates (x,y) will be stored
	// (all points with inf range value won't be considered)
	std::vector<InfoAction::Point> points;
			
	for(int i = 0; i < size; i++) {		
		// consider only the scans for which an object has been detected by the laser
		if(!isinf(msg->ranges[i]) && i > size * 1/3 && i < size * 2/3) {
			float angle = msg->angle_min + (msg->angle_increment * i); //compute the current angle of the laser

			//compute the cartesian coordinates of the point
			float x = msg->ranges[i] * cos(angle);
			float y = msg->ranges[i] * sin(angle);			
			
			points.push_back(InfoAction::Point(x,y)); // and add it to the vector
		}
	}
	
	double epsilon = 0.5; //threshold for the euclidean distance between a point and a suitable neighbour
	
	int clusterLabel = 0; //labels will be values ranging from 0 to "predicted num. of clusters" - 1

	//iterate through all the points
	for (InfoAction::Point& currentPoint : points) {
		if (!currentPoint.visited) { // check if the point hasn't been visited

			currentPoint.visited = true; //update its visited status

			// and compute all its neighbours
			std::vector<int> neighbours = InfoAction::computeNeighbours(currentPoint, points, epsilon);
			
			// starting from the found neighbours, try to expand the current cluster
			for(int i = 0; i < neighbours.size(); i++) {
				if(!points[neighbours.at(i)].visited) {
					points[neighbours.at(i)].visited = true;

					// get the neighbours of the neighbour
					std::vector<int> nextIterNeighbours = computeNeighbours(points[neighbours.at(i)], points, epsilon);

					// iterate through the indices of all the potential new neighbours
					for (int neighboursId : nextIterNeighbours) {
						// if the neighbours haven't yet been found
						if (find(neighbours.begin(), neighbours.end(), neighboursId) == neighbours.end()) {
							neighbours.push_back(neighboursId); // add it to the neighbours list
						}
					}
				}
				
				// update the label of the neighbour if it isn't already assigned to the currentPoint cluster
				if(points[neighbours.at(i)].clusterId == -1) {
					points[neighbours.at(i)].clusterId = clusterLabel;
				}
			}
			
			// the current cluster can't be expanded anymore, so update the label so that the next points will be assigned to a new one
			clusterLabel++;
		}
	}
	
	// Print the resulting people coordinates
	
	std::vector<double> x(clusterLabel); // vector that will store the sum of all the x coordinates for each cluster found
	std::vector<double> y(clusterLabel); // vector that will store the sum of all the y coordinates for each cluster found
	std::vector<int> dim(clusterLabel); // vector that will store the number of points for each cluster found
	
	for (int i = 0; i < points.size(); i++) {
		if (points[i].clusterId != -1) {
		x[points[i].clusterId] += points[i].x;
		y[points[i].clusterId] += points[i].y;
		dim[points[i].clusterId]++;
		}
	}
	
	ROS_INFO("Number of clusters: %d", clusterLabel);
	
	// Build the clusters vector
	std::vector<std::vector<InfoAction::Point>> clusters = InfoAction::buildClusters(points);

	// Compute the variance for each cluster
	std::vector<double> variances(clusterLabel);
	for(int label = 0; label < dim.size(); label++) {
		variances[label] = InfoAction::calculateClusterVariance(clusters[label]);
		ROS_INFO("Cluster %d Variance %f", label, variances[label]);
	}

	cv::Size imageSize(500, 500);

	// Plot the clusters
	std::string file_path = __FILE__;
	std::string dir_path = file_path.substr(0, file_path.rfind("/"));
	std::string filename = dir_path.append("/Clusters.jpg");
	
	InfoAction::plotClusters(clusters, imageSize, filename);
	
	// Obstacles detection with global thresholding
	ROS_INFO("Detection w/ thresholding");
	double threshold = 0.012;
	std::vector<int> obstaclesIndices = InfoAction::obstaclesDetectionThresholding(variances, threshold);
	
	// Compute the position of each obstacle (which corresponds to the center of each cluster)
	res.obstacles_positions_x.clear();
	res.obstacles_positions_y.clear();
	
	for(const int& i : obstaclesIndices) {
		res.obstacles_positions_x.push_back(x[i] / dim[i]);
		res.obstacles_positions_y.push_back(y[i] / dim[i]);
	}
}

void InfoAction::scanWithCMDCallback(const sensor_msgs::LaserScanConstPtr& msg, assignment1::InfoResult& res, float& goal_angle, float& z_rot_clockwise, float& z_rot_counterclockwise)
{
	std::vector<float> obstacles_x;
	std::vector<float> obstacles_y;
	
	// compute the size of the array of data coming from the laser scan as (angle_max - angle_mix) / angle_increment.
	int size = (msg->angle_max-msg->angle_min) / msg->angle_increment;
	
	// vector in which all points converted into cartesian coordinates (x,y) will be stored
	// (all points with inf range value won't be considered)
	std::vector<InfoAction::Point> points;
	
	float max_range_angle = msg->angle_min;
	float max_range = 0;
	
	float min_range_angle = msg->angle_max;
	float min_x;
	
	float min_range_right = std::numeric_limits<float>::max();
	float min_range_left = std::numeric_limits<float>::max();
	InfoAction::Point min_right(0.0, 0.0);
	InfoAction::Point min_left(0.0, 0.0);
			
	for(int i = 0; i < size; i++) {		
		// consider only the scans for which an object has been detected by the laser
		if(!isinf(msg->ranges[i]) && i > 20 && i < size - 20) {
			float angle = msg->angle_min + (msg->angle_increment * i); //compute the current angle of the laser

			//compute the cartesian coordinates of the point
			float x = msg->ranges[i] * cos(angle);
			float y = msg->ranges[i] * sin(angle);
			
			// calculate max range corresponding angle
			if(msg->ranges[i] > max_range) {
				max_range_angle = angle;
				max_range = msg->ranges[i];
			}
			
			// calculate closest object information on the right
			if(i > 0 && i < floor(size/2)) {
				if(msg->ranges[i] < min_range_right) {
					min_range_right = msg->ranges[i];
					min_right.x = x;
					min_right.y = y;
				}
			// calculate closest object information on the left
			} else {
				if(msg->ranges[i] < min_range_left) {
					min_range_left = msg->ranges[i];
					min_left.x = x;
					min_left.y = y;
				}
			}
			
			
			points.push_back(InfoAction::Point(x,y)); // and add it to the vector
		}
	}
	
	// if object is within collision range from the right
	if(std::abs(min_right.y) < 0.2 && std::abs(min_right.x) < 0.7) {
		// modulate the counterclockwise rotation
		z_rot_counterclockwise = std::min<float>((1/min_range_right)/5, M_PI/3);
		ROS_INFO("Obstacle RIGHT");
	}
	else
		z_rot_counterclockwise = 0.0;
	// if object is within collision range from the left
	if(std::abs(min_left.y) < 0.2 && std::abs(min_left.x) < 0.7) {
		// modulate the clockwise rotation
		z_rot_clockwise = std::max<float>((-(1/min_range_left))/5, -M_PI/3);
		ROS_INFO("Obstacle LEFT");
	}
	else
		z_rot_clockwise = 0.0;

	// modulate the desired goal angle
	goal_angle = max_range_angle / 5;
	
	//loop_rate.sleep();
	
	double epsilon = 0.5; //threshold for the euclidean distance between a point and a suitable neighbour
	
	int clusterLabel = 0; //labels will be values ranging from 0 to "predicted num. of clusters" - 1

	//iterate through all the points
	for (InfoAction::Point& currentPoint : points) {
		if (!currentPoint.visited) { // check if the point hasn't been visited

			currentPoint.visited = true; //update its visited status

			// and compute all its neighbours
			std::vector<int> neighbours = InfoAction::computeNeighbours(currentPoint, points, epsilon);
			
			// starting from the found neighbours, try to expand the current cluster
			for(int i = 0; i < neighbours.size(); i++) {
				if(!points[neighbours.at(i)].visited) {
					points[neighbours.at(i)].visited = true;

					// get the neighbours of the neighbour
					std::vector<int> nextIterNeighbours = computeNeighbours(points[neighbours.at(i)], points, epsilon);

					// iterate through the indices of all the potential new neighbours
					for (int neighboursId : nextIterNeighbours) {
						// if the neighbours haven't yet been found
						if (find(neighbours.begin(), neighbours.end(), neighboursId) == neighbours.end()) {
							neighbours.push_back(neighboursId); // add it to the neighbours list
						}
					}
				}
				
				// update the label of the neighbour if it isn't already assigned to the currentPoint cluster
				if(points[neighbours.at(i)].clusterId == -1) {
					points[neighbours.at(i)].clusterId = clusterLabel;
				}
			}
			
			// the current cluster can't be expanded anymore, so update the label so that the next points will be assigned to a new one
			clusterLabel++;
		}
	}
	
	// Print the resulting people coordinates
	
	std::vector<double> x(clusterLabel); // vector that will store the sum of all the x coordinates for each cluster found
	std::vector<double> y(clusterLabel); // vector that will store the sum of all the y coordinates for each cluster found
	std::vector<int> dim(clusterLabel); // vector that will store the number of points for each cluster found
	
	for (int i = 0; i < points.size(); i++) {
		if (points[i].clusterId != -1) {
		x[points[i].clusterId] += points[i].x;
		y[points[i].clusterId] += points[i].y;
		dim[points[i].clusterId]++;
		}
	}
	
	ROS_INFO("Number of clusters: %d", clusterLabel);
	
	// Build the clusters vector
		std::vector<std::vector<InfoAction::Point>> clusters = InfoAction::buildClusters(points);

	// Compute the variance for each cluster
	std::vector<double> variances(clusterLabel);
	for(int label = 0; label < dim.size(); label++) {
		variances[label] = InfoAction::calculateClusterVariance(clusters[label]);
		ROS_INFO("Cluster %d Variance %f", label, variances[label]);
	}

	cv::Size imageSize(500, 500);

	// Plot the clusters
	std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("/"));
	std::string filename = dir_path.append("/Clusters.jpg");
	
	// Compute the position of each obstacle (which corresponds to the center of each cluster)
	res.obstacles_positions_x.clear();
	res.obstacles_positions_y.clear();
	
	for(int i=0; i < clusterLabel; i++) {
		res.obstacles_positions_x.push_back(x[i] / dim[i]);
		res.obstacles_positions_y.push_back(y[i] / dim[i]);
	}
}

void InfoAction::cmd_vel_controller(float kp, float kd) {
	bool motion_control_law = true;
	bool end_corridor = false;

	feedback_.status.clear();
    std::string feed("The robot started moving using the motion control law");
    feedback_.status.assign(feed);
    as_.publishFeedback(feedback_);

	float goal_angle = 0.0;
	float z_rot_clockwise = 0.0;
	float z_rot_counterclockwise = 0.0;
	
	ros::Subscriber sub_robot_pose = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", 1000, boost::bind(&InfoAction::robotPoseCallback, this, _1, boost::ref(end_corridor)));

	while(!end_corridor) {
		//subscribe to the /scan topic to acquire the Laser data
		ros::Subscriber subscan = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1000, boost::bind(&InfoAction::scanWithCMDCallback, this, _1, boost::ref(result_), boost::ref(goal_angle), boost::ref(z_rot_clockwise), boost::ref(z_rot_counterclockwise)));
	
		//and publish in the /cmd_vel topic the velocities to will move the robot (in this case the linear velocity along the x axis and the angular velocity along the z axis)
		ros::Publisher chatter_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);   

		ros::Rate loop_rate(20);
		
		// previous error
		float prev_error = 0.0;
		
		// constant linear velocity
		float linear_x = 0.1;
		
		while (ros::ok() && !end_corridor) {
		//create the message that will be sent in cmd_vel to specify the robot velocities
			geometry_msgs::Twist msg;
			msg.linear.x = linear_x;
			
			// compute the total error: the desired angle and the imposed rotations constrained by the nearby obstacles
			float error = z_rot_counterclockwise + z_rot_clockwise + goal_angle;
			
			// updating motion control law for the angular speed
			msg.angular.z = kp * error - kd * (error - prev_error);
			
			ROS_INFO("z_rot_counterclockwise: %f", z_rot_counterclockwise);
			ROS_INFO("z_rot_clockwise: %f", z_rot_clockwise);
			
			prev_error = msg.angular.z;
			
		//publish the message in the topic
			chatter_pub.publish(msg);

			ros::spinOnce();
			
			//ROS_INFO("Linear vel x: %f", msg.linear.x);
			//ROS_INFO("Angular vel z: %f\n", msg.angular.z);
			
			loop_rate.sleep();
		}  	
	}

	feedback_.status.clear();
    feed = "The robot has reached the end of the corridor";
    feedback_.status.assign(feed);
    as_.publishFeedback(feedback_);
}
	
void InfoAction::executeCB(const assignment1::InfoGoalConstPtr &goal) { 
	bool motion_control_law = static_cast<bool>(goal->motion_control_law);
  	
	if(motion_control_law) {
		// proportional gain: to drive the orientation in the direction of the goal
		float kp = 1;
		
		// derivative gain: to reduce the sudden change in the angular velocity
		float kd = 0.66;

		InfoAction::cmd_vel_controller(kp, kd);
  	}

    bool success = true;
  
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
    }
    	
    ROS_INFO("Input goal position: (%f, %f), rotation along the z axis: %f", goal->pose_B_x, goal->pose_B_y, goal->pose_B_yaw);
    		
  	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  
  	//Wait for the action server of move_base to come up so that we can begin processing the goal pose
  	while(!ac.waitForServer(ros::Duration(5.0))){
    	ROS_INFO("Waiting for the move_base action server to come up");
  	}
  
  	//the ideal execution of the code uses a yaw angle of -45°, so that the robot will face
    //up left when it starts scanning to detect the position of the objects
	tf2::Quaternion quaternion;
   	quaternion.setRPY(0, 0, M_PI/180 * goal->pose_B_yaw); //Roll, Pitch, Yaw rotation
  
  	// Create a new goal to send to move_base 
  	move_base_msgs::MoveBaseGoal move_goal;

    move_goal.target_pose.header.frame_id = "map";
  	move_goal.target_pose.header.stamp = ros::Time::now();

    move_goal.target_pose.pose.position.x = goal->pose_B_x;
  	move_goal.target_pose.pose.position.y = goal->pose_B_y;

	  // Set the quaternion in the pose
    move_goal.target_pose.pose.orientation.x = quaternion.x();
    move_goal.target_pose.pose.orientation.y = quaternion.y();
    move_goal.target_pose.pose.orientation.z = quaternion.z();
    move_goal.target_pose.pose.orientation.w = quaternion.w();

    ROS_INFO("Sending goal to move_base");
    ac.sendGoal(move_goal);

	//after the goal has been sent, notify the action client that the robot is starting to move
    feedback_.status.clear();
    std::string feed("The robot started moving using move_base");
    feedback_.status.assign(feed);
    as_.publishFeedback(feedback_);

    // Wait until the robot reaches the goal
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The robot has arrived at the goal location and it stopped");
	  feedback_.status.clear();
   	  feed = "The robot has arrived at the goal location and it stopped";
	  feedback_.status.assign(feed);
      as_.publishFeedback(feedback_);
	}
    else {
      ROS_INFO("The robot failed to reach its destination and it stopped");
	  result_.obstacles_positions_x.clear();
	  result_.obstacles_positions_y.clear();
	  as_.setAborted(result_);
	  return ;
	}

	ROS_INFO("Beginning scan");

	feedback_.status.clear();
    feed = "The robot is starting the detection of the obstacles";
    feedback_.status.assign(feed);
    as_.publishFeedback(feedback_);

	ros::Duration(1.0).sleep();
	// subscribe to the /scan topic to acquire the Laser data
	ros::Subscriber subscan = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&InfoAction::scanCallback, this, _1, boost::ref(result_)));

	//to make sure that the computation of scanCallback happens we need to force to wait some time
	ros::Rate r(1);
	
  	r.sleep();

	feedback_.status.clear();
    feed = "The robot has finished the detection of the obstacles";
    feedback_.status.assign(feed);
    as_.publishFeedback(feedback_);

	if(success){    		
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
   	}
  }

int main(int argc, char** argv){
  ros::init(argc, argv, "assignment1_server");
  InfoAction info("info");

  ros::spin();
  return 0;
}
