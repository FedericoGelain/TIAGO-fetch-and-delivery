#include "B.h"

geometry_msgs::Pose DetectionAction::transform_pose(tf::StampedTransform transform, geometry_msgs::PoseWithCovarianceStamped obj_pose) {
    geometry_msgs::Pose obj_pose_original = obj_pose.pose.pose; // save the pose in the original frame (its header isn't needed)

	ROS_INFO("Pose original: (%f, %f, %f)", obj_pose_original.position.x, obj_pose_original.position.y, obj_pose_original.position.z);

    //convert the position of the pose
	tf::Vector3 v(obj_pose_original.position.x, obj_pose_original.position.y, obj_pose_original.position.z);
    
	v = transform * v;
    
    //convert the orientation of the pose
    tf::Quaternion q(obj_pose_original.orientation.x, obj_pose_original.orientation.y, obj_pose_original.orientation.z, obj_pose_original.orientation.w);
    q = transform * q;

    //create the converted pose and return it
    geometry_msgs::Pose converted_pose;

    converted_pose.position.x = v.x();
    converted_pose.position.y = v.y();
    converted_pose.position.z = v.z();
    converted_pose.orientation.x = q.x();
    converted_pose.orientation.y = q.y();
    converted_pose.orientation.z = q.z();
    converted_pose.orientation.w = q.w();

	ROS_INFO("Pose converted: (%f, %f, %f)", converted_pose.position.x, converted_pose.position.y, converted_pose.position.z);

	return converted_pose;
}

void DetectionAction::executeCB(const assignment2::DetectionGoalConstPtr &goal) { 
  
    //check that the client hasn't requested a preempt (to cancel the goal execution) and that the server hasn't shut down
    if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        return;
    }
    
    //we want to read the detections written in the topic /tag_detections of AprilTag only once.
    //Wait that a single message is published and retrieve the data
    apriltag_ros::AprilTagDetectionArrayConstPtr posesDetected = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh_);
    
    //check that the message is valid
    if(!posesDetected) {
        ROS_ERROR("Some problem occurred while trying to retrieve the april_tag detections");
		as_.setAborted(result_);
        return;
    }
    
    //extract the array of tag detections and store them into a vector
    std::vector<apriltag_ros::AprilTagDetection> pos ((*posesDetected).detections);

	//if no object was detected, then stop the execution of the callback and set the goal state to aborted
	if(pos.size() == 0) {
		ROS_ERROR("No AprilTag marker detected");
		as_.setAborted(result_);
		return;
	}

    ROS_INFO("Start processing detections..");

    //after defining the TransformListener, we wait until the transformation between the two frames is available (we want to retrieve the most recent one)
    //the frame of the detections is the same for all objects and can be accessed from the header of the pose
	try {
		tf_.waitForTransform("base_footprint", pos[0].pose.header.frame_id, ros::Time(0), ros::Duration(2));
  	    tf_.lookupTransform("base_footprint", pos[0].pose.header.frame_id, ros::Time(0), transform); //save into transform the transformation between the two frames
	}
	catch (const tf::TransformException &ex)
	{
		ROS_ERROR("Failed to transform: %s", ex.what());
	}

    //initialize all the result fields to send back to the action client
    result_.detections_poses.resize(pos.size());
	result_.detections_ids.resize(pos.size());
	result_.detections_qr_size.resize(pos.size());

	bool found = false;

    //iterate through all the objects the robot camera has seen
    for(int d = 0; d < pos.size(); d++) {		
		feedback_.status.clear();
		std::string feed ("Converting the current object pose into the robot frame");
		feedback_.status.assign(feed);
		as_.publishFeedback(feedback_);
		
        //transform the pose of the current object into the robot frame
		geometry_msgs::Pose tmp_pose_transformed = transform_pose(transform, pos[d].pose);
		
		result_.detections_poses[d] = tmp_pose_transformed;
		result_.detections_ids[d] = pos[d].id[0];
		result_.detections_qr_size[d] = pos[d].size[0];
        
		feedback_.status.clear();
		feed = "Finished converting the current object pose into the robot frame";
		feedback_.status.assign(feed);
		as_.publishFeedback(feedback_);

        //check if the current object is the one that the object has to pick
		if(pos[d].id[0] == goal->service_id) {
			feedback_.status.clear();
			feed = "The requested object was found";
			feedback_.status.assign(feed);
			as_.publishFeedback(feedback_);

			found = true;
			result_.object_found = 1;
			result_.obj_pose_to_pick = tmp_pose_transformed;
		    result_.obj_id = pos[d].id[0];
		    result_.qr_size = pos[d].size[0];
		}
    }

	if(!found)
		result_.object_found = 0;
	as_.setSucceeded(result_);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "B");

  DetectionAction detection("detection");

  ros::spin();
  return 0;
}

